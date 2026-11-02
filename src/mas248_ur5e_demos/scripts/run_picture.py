"""MAS248 run_picture: draw the strokes in /tmp/mas248_targets.json with the mock UR5e.

For every stroke: free-space move to a point above its start (MoveGroup), then one
Cartesian path down to the drawing height, along all sampled points and back up.
All strokes are preflighted in PLAN ONLY mode first, chained from each plan's end
state; nothing moves unless every stroke plans completely (L9 guide: "do not
execute partial plans"). Execution is for the bundled MOCK controllers only.
"""
import argparse
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from .common import JOINTS, move_group, cartesian, execute_trajectory

HOME=[0.0,-1.57,1.57,-1.57,-1.57,0.0]

def to_pose(p, lift=0.0):
    """JSON pose dict (world frame) -> geometry_msgs/Pose, optionally lifted along world +Z."""
    pose=Pose()
    pose.position.x=float(p['x']);pose.position.y=float(p['y']);pose.position.z=float(p['z'])+lift
    pose.orientation.x=float(p['qx']);pose.orientation.y=float(p['qy'])
    pose.orientation.z=float(p['qz']);pose.orientation.w=float(p['qw'])
    return pose

def load_strokes(path, only=None):
    data=json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
    strokes=[s['poses'] for s in data['strokes'] if len(s['poses'])>=2]
    if not strokes: raise ValueError('No strokes with at least two poses in '+str(path))
    if only is not None:
        if not 1<=only<=len(strokes): raise ValueError(f'--stroke must be 1..{len(strokes)}')
        strokes=[strokes[only-1]]
    return strokes

def end_joints(joint_trajectory):
    """Last point of a JointTrajectory as positions in common.JOINTS order."""
    if not joint_trajectory.points: return None
    named=dict(zip(joint_trajectory.joint_names,joint_trajectory.points[-1].positions))
    return [named[j] for j in JOINTS]

def stroke_waypoints(poses, approach):
    return ([to_pose(p) for p in poses]+[to_pose(poses[-1],approach)])

def plan_stroke(node, poses, approach, start, args):
    """PLAN ONLY one stroke from joint state `start` (None = current). Returns its end joints."""
    above=move_group(node,pose=to_pose(poses[0],approach),execute=False,start_joint_positions=start)
    after_approach=end_joints(above.planned_trajectory.joint_trajectory) or start
    result=cartesian(node,stroke_waypoints(poses,approach),max_step=args.max_step,
                     start_joint_positions=after_approach)
    if result.fraction<args.min_fraction:
        raise RuntimeError(f'Cartesian path only {result.fraction:.1%} complete; '
                           'move/shrink the image (--center-*, --width-m) and regenerate targets')
    return end_joints(result.solution.joint_trajectory)

def draw_stroke(node, poses, approach, args):
    """Move above the stroke, then execute its Cartesian path from the current (mock) state."""
    move_group(node,pose=to_pose(poses[0],approach),execute=True)
    result=cartesian(node,stroke_waypoints(poses,approach),max_step=args.max_step)
    if result.fraction<args.min_fraction:
        raise RuntimeError(f'Cartesian path only {result.fraction:.1%} complete at execution; stopping')
    execute_trajectory(node,result.solution)

def main():
    ap=argparse.ArgumentParser(description='Draw image targets with MoveIt. Default PLAN ONLY (preflight).')
    ap.add_argument('--json',default='/tmp/mas248_targets.json')
    ap.add_argument('--execute',action='store_true',help='Execute ONLY on the bundled mock controllers')
    ap.add_argument('--approach-mm',type=float,default=50.,help='Lift above stroke start/end (default 50 mm)')
    ap.add_argument('--stroke',type=int,default=None,help='Draw only stroke N (1-based)')
    ap.add_argument('--max-step',type=float,default=0.005,help='Cartesian interpolation step in metres')
    ap.add_argument('--min-fraction',type=float,default=0.999,help='Required Cartesian completion')
    ap.add_argument('--no-home',action='store_true',help='Skip moving to the home joint pose first')
    args=ap.parse_args()
    approach=args.approach_mm/1000.
    rclpy.init();node=Node('mas248_run_picture')
    log=node.get_logger()
    try:
        strokes=load_strokes(args.json,args.stroke)
        log.info(f'{len(strokes)} stroke(s), {sum(map(len,strokes))} points from {args.json}')
        # Preflight: chain every plan from the previous plan's end state.
        start=None
        if not args.no_home:
            start=end_joints(move_group(node,joints=HOME,execute=False).planned_trajectory.joint_trajectory) or HOME
        for i,poses in enumerate(strokes,1):
            log.info(f'Preflight stroke {i}/{len(strokes)} ({len(poses)} points)')
            start=plan_stroke(node,poses,approach,start,args)
        log.info('Preflight OK: all strokes plan completely')
        if not args.execute:
            log.warn('PLAN ONLY: the robot did not move. Run with --execute for mock drawing.')
            return
        if not args.no_home: move_group(node,joints=HOME,execute=True)
        for i,poses in enumerate(strokes,1):
            log.info(f'Drawing stroke {i}/{len(strokes)}')
            draw_stroke(node,poses,approach,args)
        if not args.no_home: move_group(node,joints=HOME,execute=True)
        log.info('Picture finished')
    except Exception as exc:
        log.error(str(exc));raise
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
