#!/usr/bin/env python3
"""MAS248 L10 one-command preflight (L10 slides 21 and 34). Never moves the robot.

  1. live stack check (validate_stack.py; --execution also requires an active
     scaled_joint_trajectory_controller and a playing External Control program)
  2. plan-only run of run_picture_real.py for the given targets (all strokes, checked)

  ros2 run mas248_ur5e_real preflight.py --targets /tmp/mas248_l10_targets_lower.json [--execution]
"""
import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--targets', '--json', dest='targets', default=None)
    ap.add_argument('--execution', action='store_true', help='Check readiness for physical execution too')
    ap.add_argument('--stroke', type=int, default=None)
    args, extra = ap.parse_known_args()
    if any(a.startswith('--exec') or a.startswith('--ack') for a in extra):
        sys.exit('preflight.py never moves the robot; use run_picture_real.py for execution')

    stack = [sys.executable, str(HERE / 'validate_stack.py')] + (['--execution'] if args.execution else [])
    print('== 1/2 Live stack check', flush=True)
    if subprocess.call(stack) != 0:
        print('PREFLIGHT FAILED in the stack check. Fix it before planning.', flush=True)
        return 1

    plan = [sys.executable, str(HERE / 'run_picture_real.py'), '--skip-stack-check']
    if args.targets:
        plan += ['--targets', args.targets]
    if args.stroke:
        plan += ['--stroke', str(args.stroke)]
    print('== 2/2 Plan-only drawing preflight', flush=True)
    if subprocess.call(plan + extra) != 0:
        print('PREFLIGHT FAILED in planning. Nothing moved.', flush=True)
        return 1
    print('PREFLIGHT PASSED. Physical execution still needs the instructor and '
          '--execute --acknowledge-hardware --acknowledge-clearance.', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
