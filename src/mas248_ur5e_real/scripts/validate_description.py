#!/usr/bin/env python3
"""MAS248 L10 offline check: render the real-robot workcell exactly like the launches do.

No robot, no network and no ROS nodes are needed. Checks: calibration file, generated
Xacro, one vendor ros2_control block, required links, SRDF/controller mapping, and
prints the TCP position at the L9 'home' pose (world frame) as a sanity value.

  ros2 run mas248_ur5e_real validate_description.py --calibration ~/ur5e_calibration.yaml
"""
import argparse
import sys
from pathlib import Path


def common_module():
    """Import launch/_real_common.py from the install share (or the source tree)."""
    candidates = [Path(__file__).resolve().parent.parent / 'launch']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, Path(get_package_share_directory('mas248_ur5e_real')) / 'launch')
    except Exception:
        pass
    for c in candidates:
        if (c / '_real_common.py').is_file():
            sys.path.insert(0, str(c))
            import _real_common
            return _real_common
    raise RuntimeError('Cannot find mas248_ur5e_real/launch/_real_common.py; build and source the workspace')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot-ip', default='192.168.1.101')
    ap.add_argument('--reverse-ip', default='192.168.1.102')
    ap.add_argument('--calibration', default=str(Path.home() / 'ur5e_calibration.yaml'))
    ap.add_argument('--controller', default='scaled_joint_trajectory_controller')
    ap.add_argument('--write-urdf', default='', help='Optional path to save the rendered URDF')
    args = ap.parse_args()

    rc = common_module()
    desc = rc.render_description(args.robot_ip, args.calibration, reverse_ip=args.reverse_ip)
    print('[OK] Calibrated real workcell renders and has one vendor ros2_control block')
    params = rc.moveit_parameters(desc, args.controller)
    print('[OK] SRDF manipulator group ends at tcp; MoveIt controller:',
          params['moveit_simple_controller_manager']['controller_names'])
    if args.write_urdf:
        Path(args.write_urdf).expanduser().write_text(desc, encoding='utf-8')
        print('[OK] URDF written to', args.write_urdf)

    try:
        print_home_tcp(desc)
    except Exception as exc:  # optional, needs only numpy
        print('[--] Skipped TCP sanity value:', exc)
    print('Description check passed.')
    return 0


def print_home_tcp(urdf_xml):
    """Forward kinematics world->tcp along the URDF chain at the SRDF 'home' joints."""
    import math
    import xml.etree.ElementTree as ET
    import numpy as np
    home = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -1.57, 'elbow_joint': 1.57,
            'wrist_1_joint': -1.57, 'wrist_2_joint': -1.57, 'wrist_3_joint': 0.0}
    root = ET.fromstring(urdf_xml)
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}

    def rpy(r, p, y):
        cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(r), math.sin(r), math.cos(p),
                                                   math.sin(p), math.cos(y), math.sin(y)))
        return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                         [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                         [-sp, cp*sr, cp*cr]])

    def axis_angle(a, t):
        a = np.array(a) / np.linalg.norm(a)
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return np.eye(3) + math.sin(t)*k + (1 - math.cos(t))*k@k

    chain, link = [], 'tcp'
    while link in by_child:
        j = by_child[link]
        chain.append(j)
        link = j.find('parent').get('link')
    T = np.eye(4)
    for j in reversed(chain):
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
        r = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
        A = np.eye(4); A[:3, :3] = rpy(*r); A[:3, 3] = xyz
        T = T @ A
        if j.get('type') in ('revolute', 'continuous'):
            ax = [float(v) for v in j.find('axis').get('xyz').split()] if j.find('axis') is not None else [1, 0, 0]
            R = np.eye(4); R[:3, :3] = axis_angle(ax, home.get(j.get('name'), 0.0))
            T = T @ R
    print('[OK] TCP at L9 home pose (world, m): x=%.3f y=%.3f z=%.3f; tool +Z axis = %s'
          % (T[0, 3], T[1, 3], T[2, 3], np.round(T[:3, 2], 2)))


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('[FAIL]', exc, file=sys.stderr)
        sys.exit(1)
