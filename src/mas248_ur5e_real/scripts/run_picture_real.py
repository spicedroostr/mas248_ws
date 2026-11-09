#!/usr/bin/env python3
"""MAS248 L10: draw image targets on the REAL UR5e (instructor-supervised).

Default is PLAN ONLY (L10 slide 34, step 7): every stroke is planned with the same
logic as the L9 run_picture (approach -> Cartesian stroke -> lift, chained end states),
then the trajectories are checked. Nothing moves.

Physical execution needs ALL of:
  --execute --acknowledge-hardware --acknowledge-clearance
  driver.launch.py ... enable_execution:=true, moveit_real.launch.py ... enable_execution:=true,
  External Control program playing on the pendant, and the instructor present.
Each trajectory is then confirmed with Enter (unless --no-confirm). Ctrl+C cancels the
running trajectory; the pendant E-stop is always the real safety device.

Every run saves evidence for the report in ~/mas248_runs/<timestamp>/:
  targets.json, plan.json (planned joint trajectories), summary.json and, when executing,
  joint_states.csv (measured joints) for planned-vs-executed plots.

  PYTHONPATH="$HOME/mas248_ws/src/mas248_ur5e_demos:$PYTHONPATH" \\
  python3 ~/mas248_ws/src/mas248_ur5e_real/scripts/run_picture_real.py \\
      --targets /tmp/mas248_l10_targets_lower.json
"""
import argparse
import csv
import json
import math
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from scripts.common import JOINTS, move_group, wait_future
    from scripts.run_picture import HOME, load_strokes, plan_stroke, plan_home, end_joints, to_pose
except ImportError as exc:  # pragma: no cover
    sys.exit('Cannot import the L9 demo code (scripts.common / scripts.run_picture): ' + str(exc) +
             '\nSource ~/mas248_ws/install/setup.bash or set PYTHONPATH to ~/mas248_ws/src/mas248_ur5e_demos')

MAX_JOINT_STEP = 0.25      # rad between two consecutive trajectory points (jump detection)
MAX_JOINT_SPEED = 0.8      # rad/s in the PLANNED timing (the pendant slider scales further down)
START_TOLERANCE = 0.02     # rad: current robot joints vs. first point of the next trajectory


def default_targets():
    for p in ('/tmp/mas248_l10_targets_lower.json', '/tmp/mas248_targets.json'):
        if Path(p).is_file():
            return p
    return '/tmp/mas248_l10_targets_lower.json'


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def check_trajectory(traj, index):
    """Return (ok, text). Time-parameterised, monotonic, no joint jumps, modest speed."""
    jt = traj.joint_trajectory
    pts = jt.points
    if len(pts) < 2:
        return True, f'trajectory {index}: {len(pts)} point(s)'
    if set(jt.joint_names) != set(JOINTS):
        return False, f'trajectory {index}: unexpected joints {list(jt.joint_names)}'
    times = [seconds(p.time_from_start) for p in pts]
    if times[-1] <= 0.0 or any(b <= a for a, b in zip(times, times[1:])):
        return False, f'trajectory {index}: not time-parameterised / non-monotonic time'
    max_step = max_speed = 0.0
    for a, b, ta, tb in zip(pts, pts[1:], times, times[1:]):
        step = max(abs(x - y) for x, y in zip(a.positions, b.positions))
        max_step = max(max_step, step)
        max_speed = max(max_speed, step / (tb - ta))
    if max_step > MAX_JOINT_STEP:
        return False, f'trajectory {index}: joint jump {max_step:.3f} rad between two points (IK flip?)'
    if max_speed > MAX_JOINT_SPEED:
        return False, f'trajectory {index}: planned joint speed {max_speed:.2f} rad/s > {MAX_JOINT_SPEED}'
    return True, (f'trajectory {index}: {len(pts)} points, {times[-1]:.1f} s, '
                  f'max step {max_step:.3f} rad, max speed {max_speed:.2f} rad/s')


def traj_to_dict(traj):
    jt = traj.joint_trajectory
    return {'joint_names': list(jt.joint_names),
            'points': [{'t': seconds(p.time_from_start), 'positions': list(p.positions)}
                       for p in jt.points]}


class Recorder:
    """Stores /joint_states while trajectories execute (callbacks run while we spin)."""

    def __init__(self, node):
        self.rows = []
        self.active_traj = 0
        self.last = None
        node.create_subscription(JointState, '/joint_states', self.cb, 50)

    def cb(self, msg):
        named = dict(zip(msg.name, msg.position))
        if not set(JOINTS).issubset(named):
            return
        self.last = [named[j] for j in JOINTS]
        if self.active_traj:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.rows.append([f'{stamp:.4f}', self.active_traj] + [f'{v:.6f}' for v in self.last])

    def save(self, path):
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['stamp', 'trajectory'] + list(JOINTS))
            w.writerows(self.rows)


def current_joints(node, recorder, timeout=3.0):
    end = time.monotonic() + timeout
    recorder.last = None
    while recorder.last is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return recorder.last


def execute(node, traj, recorder, index):
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import MoveItErrorCodes
    cli = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    if not cli.wait_for_server(timeout_sec=10.0):
        raise RuntimeError('/execute_trajectory unavailable: start moveit_real.launch.py with enable_execution:=true')
    goal = ExecuteTrajectory.Goal()
    goal.trajectory = traj
    handle = wait_future(node, cli.send_goal_async(goal), 30)
    if not handle.accepted:
        raise RuntimeError('ExecuteTrajectory goal rejected')
    recorder.active_traj = index
    t0 = time.monotonic()
    try:
        result = wait_future(node, handle.get_result_async(), 600).result
    except KeyboardInterrupt:
        node.get_logger().error('Ctrl+C: cancelling the running trajectory')
        wait_future(node, handle.cancel_goal_async(), 5)
        raise
    finally:
        recorder.active_traj = 0
    wall = time.monotonic() - t0
    if result.error_code.val != MoveItErrorCodes.SUCCESS:
        hints = {-4: 'CONTROL_FAILED: check pendant, protective stop, program running',
                 -6: 'TIMED_OUT: robot slower than planned (speed slider) or not moving '
                     '(External Control program not playing)'}
        code = result.error_code.val
        raise RuntimeError(f'Trajectory {index} failed after {time.monotonic() - t0:.1f} s, '
                           f'MoveItErrorCodes={code} ({hints.get(code, "see move_group log")})')
    return wall


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--targets', '--json', dest='targets', default=default_targets())
    ap.add_argument('--execute', action='store_true', help='Move the REAL robot (needs both acknowledgements)')
    ap.add_argument('--acknowledge-hardware', action='store_true', help='I know this moves the real UR5e')
    ap.add_argument('--acknowledge-clearance', action='store_true',
                    help='Workspace is clear, E-stop in reach, instructor present')
    ap.add_argument('--no-confirm', action='store_true', help='Do not ask for Enter before each trajectory')
    ap.add_argument('--approach-mm', type=float, default=50.)
    ap.add_argument('--stroke', type=int, default=None, help='Only stroke N (1-based); good for the first test')
    ap.add_argument('--max-step', type=float, default=0.005)
    ap.add_argument('--jump-threshold', type=float, default=0.0)
    ap.add_argument('--min-fraction', type=float, default=0.999)
    ap.add_argument('--attempts', type=int, default=5)
    ap.add_argument('--max-joint-jump', type=float, default=0.25)
    ap.add_argument('--max-joint-travel', type=float, default=3.3,
                    help='Reject free-space plans where one joint turns more than this (rad); 0 = off')
    ap.add_argument('--no-home', action='store_true')
    ap.add_argument('--skip-stack-check', action='store_true', help='Planning only: skip live driver checks')
    ap.add_argument('--log-dir', default=str(Path.home() / 'mas248_runs'))
    args = ap.parse_args()

    if args.execute and not (args.acknowledge_hardware and args.acknowledge_clearance):
        sys.exit('Refusing: --execute needs --acknowledge-hardware and --acknowledge-clearance '
                 '(instructor-supervised physical execution).')

    run_dir = Path(args.log_dir).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir.mkdir(parents=True, exist_ok=True)
    summary = {'targets': args.targets, 'mode': 'execute' if args.execute else 'plan_only',
               'started': datetime.now().isoformat(timespec='seconds'), 'args': vars(args)}

    rclpy.init()
    node = Node('mas248_run_picture_real')
    log = node.get_logger()
    recorder = Recorder(node)
    try:
        # Live checks (never move). Planning-only tolerates an inactive controller / stopped program.
        if not args.skip_stack_check:
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            from validate_stack import StackChecker
            if not StackChecker(node).run(execution=args.execute):
                raise RuntimeError('Stack check failed (see [FAIL] lines above)')

        shutil.copy2(Path(args.targets).expanduser(), run_dir / 'targets.json')
        strokes = load_strokes(args.targets, args.stroke)
        zs = [float(p['z']) for s in strokes for p in s]
        log.info(f'{len(strokes)} stroke(s), {sum(map(len, strokes))} points, target z '
                 f'{min(zs):.3f}..{max(zs):.3f} m (world) from {args.targets}')

        # ---- Preflight: plan everything first, chained from each plan's end state ----
        approach = args.approach_mm / 1000.
        plan, start = [], None
        t_plan = time.monotonic()
        if not args.no_home:
            trajs, start = plan_home(node, None, args)
            plan += trajs
        travel_from = None
        for i, poses in enumerate(strokes, 1):
            log.info(f'Preflight stroke {i}/{len(strokes)} ({len(poses)} points)')
            trajs, start = plan_stroke(node, poses, approach, start, args, travel_from)
            travel_from = to_pose(poses[-1], approach)
            plan += trajs
        if not args.no_home:
            trajs, start = plan_home(node, start, args, lift_from=travel_from)
            plan += trajs
        summary['planning_time_s'] = round(time.monotonic() - t_plan, 2)
        log.info(f'Planning OK: {len(plan)} trajectories in {summary["planning_time_s"]} s')
        log.info('Collisions OK: MoveIt collision-checked every planned state (OMPL + Cartesian avoid_collisions)')

        checks = [check_trajectory(t, k) for k, t in enumerate(plan, 1)]
        for ok, text in checks:
            (log.info if ok else log.error)(('  ' if ok else 'BAD ') + text)
        (run_dir / 'plan.json').write_text(json.dumps([traj_to_dict(t) for t in plan]), encoding='utf-8')
        summary['trajectories'] = [{'index': k, 'check': text, 'ok': ok,
                                    'planned_s': round(seconds(t.joint_trajectory.points[-1].time_from_start), 3)
                                    if t.joint_trajectory.points else 0.0}
                                   for k, ((ok, text), t) in enumerate(zip(checks, plan), 1)]
        if not all(ok for ok, _ in checks):
            raise RuntimeError('Trajectory check failed: do not execute; regenerate/move the targets')
        log.info('Trajectory valid')
        log.info('Preflight check passed!')
        summary['preflight'] = 'passed'

        if not args.execute:
            log.warn('PLAN ONLY: the robot did not move. Evidence saved in ' + str(run_dir))
            return 0

        # ---- Physical execution (instructor-supervised) ----
        now = current_joints(node, recorder)
        first = plan[0].joint_trajectory
        named = dict(zip(first.joint_names, first.points[0].positions))
        if now is None:
            raise RuntimeError('No /joint_states from the robot')
        diff = max(abs(a - named[j]) for a, j in zip(now, JOINTS))
        if diff > START_TOLERANCE:
            raise RuntimeError(f'Robot moved since planning ({diff:.3f} rad): run again to re-plan')
        for k, traj in enumerate(plan, 1):
            if not args.no_confirm:
                answer = input(f'Execute trajectory {k}/{len(plan)} '
                               f'({summary["trajectories"][k-1]["planned_s"]} s planned)? '
                               '[Enter = go, q = stop] ')
                if answer.strip().lower().startswith('q'):
                    log.warn('Stopped by user before trajectory %d' % k)
                    summary['stopped_before'] = k
                    break
            wall = execute(node, traj, recorder, k)
            summary['trajectories'][k-1]['executed_wall_s'] = round(wall, 3)
            log.info(f'Trajectory {k}/{len(plan)} done in {wall:.1f} s')
        else:
            log.info('Picture finished on the real robot')
            summary['finished'] = True
        return 0
    except KeyboardInterrupt:
        summary['error'] = 'interrupted'
        log.error('Interrupted')
        return 130
    except Exception as exc:
        summary['error'] = str(exc)
        log.error(str(exc))
        return 1
    finally:
        summary['ended'] = datetime.now().isoformat(timespec='seconds')
        if recorder.rows:
            recorder.save(run_dir / 'joint_states.csv')
        (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print('Evidence:', run_dir, flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
