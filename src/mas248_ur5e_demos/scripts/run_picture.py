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
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from .common import JOINTS, move_group, cartesian, execute_trajectory

HOME=[0.0,-1.57,1.57,-1.57,-1.57,0.0]
TWO_PI=2*math.pi
# UR5e position limits: all joints +-2pi except the elbow (+-pi)
LIMITS=[TWO_PI,TWO_PI,math.pi,TWO_PI,TWO_PI,TWO_PI]

def unwrap(target, seed):
    """Shift every joint of `target` by k*2pi to the equivalent angle closest to `seed`
    (same TCP pose, but no 360-degree turn), staying inside the UR5e joint limits."""
    out=[]
    for t,s,lim in zip(target,seed,LIMITS):
        best=t
        for k in (-2,-1,1,2):
            c=t+k*TWO_PI
            if abs(c)<=lim and abs(c-s)<abs(best-s): best=c
        out.append(best)
    return out

def current_joints(node, timeout=3.0):
    """One /joint_states sample in JOINTS order (None if nothing arrives)."""
    import time
    from sensor_msgs.msg import JointState
    got={}
    def cb(msg):
        named=dict(zip(msg.name,msg.position))
        if all(j in named for j in JOINTS): got['q']=[named[j] for j in JOINTS]
    sub=node.create_subscription(JointState,'/joint_states',cb,10)
    end=time.monotonic()+timeout
    while 'q' not in got and time.monotonic()<end: rclpy.spin_once(node,timeout_sec=0.05)
    node.destroy_subscription(sub)
    return got.get('q')

def seeded_ik(node, pose, seed):
    """IK for the tcp at `pose` (world), seeded with joint state `seed`, collision-checked.
    KDL starts from the seed, so it normally returns the SAME arm/wrist branch; unwrap()
    then removes any remaining 2pi offsets. Returns joints or None."""
    from moveit_msgs.srv import GetPositionIK
    from geometry_msgs.msg import PoseStamped
    cli=node.create_client(GetPositionIK,'/compute_ik')
    if not cli.wait_for_service(timeout_sec=10.0): return None
    req=GetPositionIK.Request()
    r=req.ik_request;r.group_name='manipulator';r.ik_link_name='tcp';r.avoid_collisions=True
    r.robot_state.joint_state.name=list(JOINTS);r.robot_state.joint_state.position=[float(v) for v in seed]
    ps=PoseStamped();ps.header.frame_id='world';ps.pose=pose;r.pose_stamped=ps
    r.timeout.sec=0;r.timeout.nanosec=200_000_000
    fut=cli.call_async(req);rclpy.spin_until_future_complete(node,fut,timeout_sec=10.0)
    res=fut.result()
    if res is None or res.error_code.val!=1: return None
    named=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    if not all(j in named for j in JOINTS): return None
    return unwrap([named[j] for j in JOINTS],seed)

def joint_travel(joint_trajectory):
    """Largest total rotation of one joint along the trajectory (rad, summed |steps|)."""
    pts=joint_trajectory.points
    if len(pts)<2: return 0.0
    n=len(pts[0].positions)
    return max(sum(abs(q.positions[i]-p.positions[i]) for p,q in zip(pts,pts[1:])) for i in range(n))

def state_valid(node, joints):
    """Collision/limit check of one joint state via move_group's /check_state_validity."""
    from moveit_msgs.srv import GetStateValidity
    cli=node.create_client(GetStateValidity,'/check_state_validity')
    if not cli.wait_for_service(timeout_sec=10.0): raise RuntimeError('/check_state_validity unavailable')
    req=GetStateValidity.Request();req.group_name='manipulator'
    req.robot_state.joint_state.name=list(JOINTS);req.robot_state.joint_state.position=[float(v) for v in joints]
    fut=cli.call_async(req);rclpy.spin_until_future_complete(node,fut,timeout_sec=10.0)
    res=fut.result()
    return res is not None and res.valid

def straight_joint_move(node, start, goal, vmax=0.3, step=0.02):
    """Deterministic free move: every joint goes the SHORT, straight way from `start` to `goal`
    in joint space (like a UR 'MoveJ' / Pilz PTP), with a smooth cosine speed profile, all
    joints starting and stopping together. Every intermediate state (<= `step` rad apart) is
    collision-checked by MoveIt. Returns a RobotTrajectory, or None if a state collides."""
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    d=[g-s0 for s0,g in zip(start,goal)]
    dmax=max(abs(x) for x in d)
    traj=RobotTrajectory();traj.joint_trajectory.joint_names=list(JOINTS)
    if dmax<1e-4: return traj
    n=max(2,int(math.ceil(dmax/step)))
    T=max(1.0,(math.pi/2)*dmax/vmax)          # peak speed of the cosine profile = vmax
    for i in range(n+1):
        u=i/n
        q=[s0+di*u for s0,di in zip(start,d)]
        if 0<i<n and not state_valid(node,q): return None
        t=T*math.acos(1-2*u)/math.pi          # time where the cosine profile reaches fraction u
        pt=JointTrajectoryPoint();pt.positions=q
        sdot=(math.pi/(2*T))*math.sin(math.pi*t/T);sdd=(math.pi**2/(2*T*T))*math.cos(math.pi*t/T)
        pt.velocities=[di*sdot for di in d];pt.accelerations=[di*sdd for di in d]
        pt.time_from_start=Duration(sec=int(t),nanosec=int((t-int(t))*1e9))
        traj.joint_trajectory.points.append(pt)
    return traj

def plan_joint_goal(node, goal, start, args, what):
    """OMPL plan to a JOINT goal (fixed end configuration, no random IK branch). Retries,
    and rejects plans where a joint turns more than --max-joint-travel (no 360 spins)."""
    log=node.get_logger();limit=getattr(args,'max_joint_travel',3.3);last=None
    q0=start if start is not None else current_joints(node)
    if q0 is not None:
        change=max(abs(g-s0) for s0,g in zip(q0,goal))
        if limit>0 and change>limit:
            raise RuntimeError(f'{what}: a joint would turn {change:.2f} rad (> --max-joint-travel {limit}); '
                               'jog the robot closer to home first')
        traj=straight_joint_move(node,q0,goal)
        if traj is not None:
            log.info(f'{what}: straight joint-space move, largest joint change '
                     f'{max(abs(g-s0) for s0,g in zip(q0,goal)):.2f} rad')
            return traj
        log.warn(f'{what}: straight joint-space move would collide; falling back to OMPL')
    for attempt in range(1,args.attempts+1):
        try:
            traj=move_group(node,joints=goal,execute=False,start_joint_positions=start).planned_trajectory
        except RuntimeError as exc:
            last=exc;log.warn(f'{what} attempt {attempt}/{args.attempts} failed ({exc}); retrying');continue
        travel=joint_travel(traj.joint_trajectory)
        if limit>0 and travel>limit:
            last=f'a joint turns {travel:.2f} rad';log.warn(f'{what} attempt {attempt}: {last} (> {limit}); re-planning');continue
        return traj
    raise RuntimeError(f'Could not plan {what} after {args.attempts} attempts: {last}. '
                       'If the robot starts far from home, jog it closer first (or --max-joint-travel 0 to disable the check)')

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
    seed=start if start is not None else current_joints(node)
    goal=unwrap(HOME,seed) if seed is not None else HOME
    home=plan_joint_goal(node,goal,start,args,'Home')
    if home.joint_trajectory.points:trajs.append(home)
    return trajs,end_joints(home.joint_trajectory) or start

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
            seed=start if start is not None else current_joints(node)
            goal=seeded_ik(node,above_pose,seed) if (seed is not None and attempt<args.attempts) else None
            try:
                if goal is not None:
                    # Joint-space goal from IK seeded with the current joints: same IK branch,
                    # nearest 2pi-equivalent angles => no wrist/base spins on the approach
                    traj=plan_joint_goal(node,goal,start,args,'Approach')
                else:
                    log.warn(f'Attempt {attempt}: no seeded IK; using OMPL pose goal (random IK branch)')
                    traj=move_group(node,pose=above_pose,execute=False,start_joint_positions=start).planned_trajectory
                    limit=getattr(args,'max_joint_travel',3.3)
                    if limit>0 and joint_travel(traj.joint_trajectory)>limit:
                        log.warn(f'Attempt {attempt}: pose-goal approach turns a joint {joint_travel(traj.joint_trajectory):.2f} rad; re-planning')
                        continue
            except RuntimeError as exc:
                # OMPL is random: a failed approach (-1, -2, 99999) is re-planned, not fatal
                log.warn(f'Attempt {attempt}: approach planning failed ({exc}); re-planning')
                continue
            after_approach=end_joints(traj.joint_trajectory) or start
            trajs=[traj] if traj.joint_trajectory.points else []
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
    ap.add_argument('--max-joint-travel',type=float,default=3.3,help='Reject free-space plans where one joint turns more than this (rad); 0 = off')
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
