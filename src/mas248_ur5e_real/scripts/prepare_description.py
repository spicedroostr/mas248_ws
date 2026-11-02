#!/usr/bin/env python3
"""MAS248 L10: derive the real-hardware workcell Xacro from the canonical L8 workcell.

Called by CMakeLists.txt at configure time. Writes ONLY into the build directory:
  --output         build/.../urdf/ur5e_workcell_real.urdf.xacro
  --config-output  build/.../config/ur5e/  (vendor YAML copied from ur_description)
                   build/.../config/ur5e_update_rate.yaml (from ur_robot_driver, if found)

What changes compared with the L8 workcell (geometry, pedestal, spindle and TCP are untouched):
  * the <xacro:ur_robot> call gets generate_ros2_control_tag=true, so the vendor
    ur_robot_driver/URPositionHardwareInterface is the ONE ros2_control system;
  * every driver/hardware value (robot_ip, reverse_ip, ports, calibration, recipes, ...)
    becomes a top-level xacro:arg, so ur_control.launch.py and _real_common.py can set it;
  * use_fake_hardware defaults to false and safety_limits to true.
Only parameters that the INSTALLED ur_description macro accepts are forwarded, so the
generator follows whatever Humble release is installed on the lab computer.
"""
import argparse
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

XACRO_NS = 'http://www.ros.org/wiki/xacro'
X = '{%s}' % XACRO_NS
HERE = Path(__file__).resolve().parent
DEFAULTS_YAML = HERE.parent / 'config' / 'real_robot_defaults.yaml'

# macro parameter -> top-level xacro arg (same names as ur_control.launch.py passes)
ARG_FOR_PARAM = {
    'name': 'name', 'ur_type': 'ur_type', 'tf_prefix': 'tf_prefix',
    'joint_limits_parameters_file': 'joint_limit_params',
    'kinematics_parameters_file': 'kinematics_params',
    'physical_parameters_file': 'physical_params',
    'visual_parameters_file': 'visual_params',
    'use_fake_hardware': 'use_fake_hardware',
    'fake_sensor_commands': 'fake_sensor_commands',
    'robot_ip': 'robot_ip',
    'script_filename': 'script_filename',
    'input_recipe_filename': 'input_recipe_filename',
    'output_recipe_filename': 'output_recipe_filename',
    'use_currents_as_efforts': 'use_currents_as_efforts',
}
# Parameters that MUST exist in the installed macro, or the real driver cannot work.
REQUIRED_PARAMS = {'robot_ip', 'kinematics_parameters_file', 'script_filename',
                   'input_recipe_filename', 'output_recipe_filename', 'generate_ros2_control_tag'}

# Defaults for args that are not in real_robot_defaults.yaml.
EXTRA_ARG_DEFAULTS = {
    'ur_type': 'ur5e', 'name': 'ur5e', 'tf_prefix': '', 'robot_ip': '0.0.0.0',
    'use_fake_hardware': 'false', 'fake_sensor_commands': 'false', 'mock_control': 'false',
    'use_currents_as_efforts': 'true',
    # Empty on purpose: the real robot must be given its calibration file.
    'kinematics_params': '',
    'joint_limit_params': '$(find ur_description)/config/$(arg ur_type)/joint_limits.yaml',
    'physical_params': '$(find ur_description)/config/$(arg ur_type)/physical_parameters.yaml',
    'visual_params': '$(find ur_description)/config/$(arg ur_type)/visual_parameters.yaml',
    'script_filename': '', 'input_recipe_filename': '', 'output_recipe_filename': '',
}
FORCED_DEFAULTS = {'use_fake_hardware': 'false', 'safety_limits': 'true'}


def share(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(pkg))
    except Exception:
        fallback = Path('/opt/ros/humble/share') / pkg
        if fallback.is_dir():
            return fallback
        raise RuntimeError(f'Package {pkg} not found. Source /opt/ros/humble/setup.bash '
                           f'and install ros-humble-ur (sudo apt install ros-humble-ur)')


def macro_params(ur_macro):
    text = ur_macro.read_text(encoding='utf-8')
    m = re.search(r'<xacro:macro\s+name="ur_robot"\s+params="([^"]*)"', text, re.S)
    if not m:
        raise RuntimeError('Cannot find <xacro:macro name="ur_robot"> in ' + str(ur_macro))
    names = set()
    for token in m.group(1).split():
        names.add(token.split(':=')[0].split('=')[0].lstrip('*'))
    return names


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--config-output', required=True)
    args = ap.parse_args()

    src = Path(args.source)
    out = Path(args.output)
    cfg_out = Path(args.config_output)
    hw_defaults = {k: (str(v).lower() if isinstance(v, bool) else str(v))
                   for k, v in yaml.safe_load(DEFAULTS_YAML.read_text(encoding='utf-8'))['hardware'].items()}

    ur_desc = share('ur_description')
    supported = macro_params(ur_desc / 'urdf' / 'ur_macro.xacro')
    missing = REQUIRED_PARAMS - supported
    if missing:
        raise RuntimeError('Installed ur_description is too old, ur_robot lacks: ' + ', '.join(sorted(missing)))

    ET.register_namespace('xacro', XACRO_NS)
    tree = ET.parse(str(src), parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    robot = tree.getroot()
    if robot.findall('.//ros2_control'):
        raise RuntimeError('The L8 workcell must not contain <ros2_control> (that is the L9 mock wrapper)')
    calls = robot.findall('.//' + X + 'ur_robot')
    if len(calls) != 1:
        raise RuntimeError(f'Expected exactly one <xacro:ur_robot> in {src}, found {len(calls)}')
    call = calls[0]

    # Hardware args: same name as macro parameter (reverse_ip, ports, safety, tool_*, ...)
    mapping = dict(ARG_FOR_PARAM)
    mapping.update({k: k for k in hw_defaults})
    used_args, ignored = set(), []
    for param, arg in sorted(mapping.items()):
        if param in supported:
            call.set(param, '$(arg %s)' % arg)
            used_args.add(arg)
        elif arg in hw_defaults:
            ignored.append(param)
    call.set('generate_ros2_control_tag', 'true')

    # Declare every arg at top level (_real_common passes them all; unknown ones are harmless).
    declared = {e.get('name'): e for e in robot.findall(X + 'arg')}
    wanted = {**EXTRA_ARG_DEFAULTS, **hw_defaults, **FORCED_DEFAULTS}
    for name, elem in declared.items():
        if name in FORCED_DEFAULTS:
            elem.set('default', FORCED_DEFAULTS[name])
    insert_at = max([list(robot).index(e) for e in declared.values()] or [-1]) + 1
    for name in sorted(set(wanted) - set(declared)):
        elem = ET.Element(X + 'arg', {'name': name, 'default': wanted[name]})
        elem.tail = '\n  '
        robot.insert(insert_at, elem)
        insert_at += 1
    robot.insert(0, ET.Comment(
        ' GENERATED by mas248_ur5e_real/scripts/prepare_description.py from ' + src.name +
        '. Do not edit: change the L8 workcell and rebuild. '))

    out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(out), encoding='utf-8', xml_declaration=True)

    cfg_out.mkdir(parents=True, exist_ok=True)
    for name in ('joint_limits.yaml', 'physical_parameters.yaml', 'visual_parameters.yaml',
                 'default_kinematics.yaml'):
        shutil.copy2(ur_desc / 'config' / 'ur5e' / name, cfg_out / name)
    copied_rate = 'source copy'
    try:
        rate = share('ur_robot_driver') / 'config' / 'ur5e_update_rate.yaml'
        if rate.is_file():
            shutil.copy2(rate, cfg_out.parent / 'ur5e_update_rate.yaml')
            copied_rate = str(rate)
    except RuntimeError:
        pass
    print(f'L10 workcell generated: {out} (ur_description {ur_desc}); '
          f'update rate: {copied_rate}' +
          (f'; not supported by installed macro, ignored: {", ".join(ignored)}' if ignored else ''))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('prepare_description.py: ' + str(exc), file=sys.stderr)
        sys.exit(1)
