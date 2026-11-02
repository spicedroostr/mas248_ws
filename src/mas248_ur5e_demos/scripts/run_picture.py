"""MAS248 run_picture: draw the strokes in /tmp/mas248_targets.json with the mock UR5e.

For every stroke: free-space move to a point above its start (MoveGroup), then one
Cartesian path down to the drawing height, along all sampled points and back up.
All strokes are preflighted in PLAN ONLY mode first, chained from each plan's end
state; nothing moves unless every stroke plans completely (L9 guide: "do not
execute partial plans"). --execute then runs exactly the preflighted trajectories
on the bundled MOCK controllers only.
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

def joint_jump(joint_trajectory):
    """Largest single-joint change between two consecutive trajectory points (rad)."""
    pts=joint_trajectory.points
    return max((max(abs(a-b) for a,b in zip(p.positions,q.positions)) for p,q in zip(pts,pts[1:])),default=0.0)

def plan_home(node, start, args, lift_from=None, lift=0.15):
    """PLAN ONLY a move to HOME from joint state `start` (None = current).

    From the last stroke the pen first rises `lift` metres straight up (Cartesian, same IK
    branch) so OMPL does not start squeezed between table and arm, then OMPL plans to HOME.
    OMPL is random and sometimes gives up (MoveItErrorCodes 99999 / -1), so it is retried
    up to --attempts times. Returns (trajectories, end joints)."""
    log=node.get_logger();trajs=[]
    if lift_from is not None:
        up=Pose();up.position.x=lift_from.position.x;up.position.y=lift_from.position.y
        up.position.z=lift_from.position.z+lift;up.orientation=lift_from.orientation
        res=cartesian(node,[lift_from,up],max_step=args.max_step,jump_threshold=args.jump_threshold,
                      start_joint_positions=start)
        if res.fraction>=args.min_fraction and joint_jump(res.solution.joint_trajectory)<=getattr(args,'max_joint_jump',0.3):
            trajs.append(res.solution);start=end_joints(res.solution.joint_trajectory) or start
        else:
            log.warn(f'Lift before home only {res.fraction:.1%}; planning home directly')
    last=None
    for attempt in range(1,args.attempts+1):
        try:
            home=move_group(node,joints=HOME,execute=False,start_joint_positions=start).planned_trajectory
            if home.joint_trajectory.points:trajs.append(home)
            return trajs,end_joints(home.joint_trajectory) or start
        except RuntimeError as exc:
            last=exc;log.warn(f'Home attempt {attempt}/{args.attempts} failed ({exc}); retrying')
    raise RuntimeError(f'Could not plan to HOME after {args.attempts} attempts: {last}')

def plan_stroke(node, poses, approach, start, args, travel_from=None):
    """PLAN ONLY one stroke from joint state `start` (None = current).

    Approach: if `travel_from` (the lifted end pose of the previous stroke) is given, the
    pen first travels in a straight Cartesian line at lift height to above this stroke's
    start. Cartesian IK is seeded with the current joints, so the arm stays on the same
    IK branch: no wrist spins / 'dance' between strokes. Only if that fails (or for the
    first stroke) is the approach planned with OMPL (MoveGroup pose goal, random IK branch,
    re-planned up to --attempts times).
    Every Cartesian part is rejected if one joint jumps more than --max-joint-jump between
    two points (MoveIt's own jump test is off, see --jump-threshold).
    Returns (trajectories to execute, end joints)."""
    log=node.get_logger()
    max_jump=getattr(args,'max_joint_jump',0.3)
    above_pose=to_pose(poses[0],approach)
    best=0.0
    for attempt in range(1,args.attempts+1):
        trajs=[];after_approach=None
        if travel_from is not None and attempt==1:
            travel=cartesian(node,[travel_from,above_pose],max_step=args.max_step,
                             jump_threshold=args.jump_threshold,start_joint_positions=start)
            jump=joint_jump(travel.solution.joint_trajectory)
            if travel.fraction>=args.min_fraction and jump<=max_jump:
                trajs=[travel.solution]
                after_approach=end_joints(travel.solution.joint_trajectory) or start
            else:
                log.warn(f'Cartesian travel {travel.fraction:.1%} (max joint jump {jump:.2f} rad); using OMPL approach')
        if not trajs:
            above=move_group(node,pose=above_pose,execute=False,start_joint_positions=start)
            after_approach=end_joints(above.planned_trajectory.joint_trajectory) or start
            trajs=[t for t in (above.planned_trajectory,) if t.joint_trajectory.points]
        result=cartesian(node,stroke_waypoints(poses,approach),max_step=args.max_step,
                         jump_threshold=args.jump_threshold,start_joint_positions=after_approach)
        jump=joint_jump(result.solution.joint_trajectory)
        if result.fraction>=args.min_fraction and jump<=max_jump:
            return trajs+[result.solution],end_joints(result.solution.joint_trajectory)
        best=max(best,result.fraction)
        log.warn(f'Attempt {attempt}: Cartesian path {result.fraction:.1%}, max joint jump {jump:.2f} rad; re-planning approach')
    raise RuntimeError(f'Cartesian path at best {best:.1%} complete after {args.attempts} attempts; '
                       'move/shrink the image (--center-*, --width-m) and regenerate targets')

def main():
    ap=argparse.ArgumentParser(description='Draw image targets with MoveIt. Default PLAN ONLY (preflight).')
    ap.add_argument('--json',default='/tmp/mas248_targets.json')
    ap.add_argument('--execute',action='store_true',help='Execute ONLY on the bundled mock controllers')
    ap.add_argument('--approach-mm',type=float,default=50.,help='Lift above stroke start/end (default 50 mm)')
    ap.add_argument('--stroke',type=int,default=None,help='Draw only stroke N (1-based)')
    ap.add_argument('--max-step',type=float,default=0.005,help='Cartesian interpolation step in metres')
    # MoveGroup only reaches the approach pose within 8 mm / 0.1 rad, so the first Cartesian
    # step corrects that error and Humble's relative jump test (e.g. 1.5) cuts the path there.
    ap.add_argument('--jump-threshold',type=float,default=0.0,help='Relative joint jump limit, 0 = off')
    ap.add_argument('--min-fraction',type=float,default=0.999,help='Required Cartesian completion')
    ap.add_argument('--attempts',type=int,default=5,help='Approach re-plans per stroke before giving up')
    ap.add_argument('--max-joint-jump',type=float,default=0.3,help='Reject Cartesian paths where a joint moves more than this (rad) between two points')
    ap.add_argument('--no-home',action='store_true',help='Skip moving to the home joint pose first')
    args=ap.parse_args()
    approach=args.approach_mm/1000.
    rclpy.init();node=Node('mas248_run_picture')
    log=node.get_logger()
    try:
        strokes=load_strokes(args.json,args.stroke)
        log.info(f'{len(strokes)} stroke(s), {sum(map(len,strokes))} points from {args.json}')
        # Preflight: plan everything chained from the previous plan's end state, then
        # execute exactly those trajectories (re-planning could pick another IK branch).
        plan=[];start=None
        if not args.no_home:
            trajs,start=plan_home(node,None,args)
            plan+=trajs
        travel_from=None
        for i,poses in enumerate(strokes,1):
            log.info(f'Preflight stroke {i}/{len(strokes)} ({len(poses)} points)')
            trajs,start=plan_stroke(node,poses,approach,start,args,travel_from)
            travel_from=to_pose(poses[-1],approach)
            plan+=trajs
        if not args.no_home:
            trajs,start=plan_home(node,start,args,lift_from=travel_from)
            plan+=trajs
        log.info(f'Preflight OK: all strokes plan completely ({len(plan)} trajectories)')
        if not args.execute:
            log.warn('PLAN ONLY: the robot did not move. Run with --execute for mock drawing.')
            return
        for k,traj in enumerate(plan,1):
            log.info(f'Executing trajectory {k}/{len(plan)}')
            execute_trajectory(node,traj)
        log.info('Picture finished')
    except Exception as exc:
        log.error(str(exc));raise
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
