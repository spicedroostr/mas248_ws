#!/usr/bin/env python3
"""MAS248 L10 live checks (L10 slides 16 and 21) BEFORE the first motion. Never moves the robot.

  1. /joint_states is fresh and has the six UR5e joints
  2. controller_manager: joint_state_broadcaster active, trajectory controller loaded
     (active only when execution is enabled)
  3. MoveIt: /move_action, /compute_cartesian_path and /execute_trajectory exist
  4. Robot: io_and_status_controller/robot_program_running == true (External Control playing),
     plus robot mode, safety mode and speed scaling when available

  ros2 run mas248_ur5e_real validate_stack.py              # planning-only checks
  ros2 run mas248_ur5e_real validate_stack.py --execution  # also require execution readiness
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class StackChecker:
    def __init__(self, node, controller='scaled_joint_trajectory_controller'):
        self.node = node
        self.controller = controller
        self.results = []   # (ok, required, text)
        self.latest = {}
        node.create_subscription(JointState, '/joint_states', self._store('joint_states'), 10)
        node.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                 self._store('program'), LATCHED)
        node.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                                 self._store('speed'), 10)
        try:
            from ur_dashboard_msgs.msg import RobotMode, SafetyMode
            node.create_subscription(RobotMode, '/io_and_status_controller/robot_mode',
                                     self._store('robot_mode'), LATCHED)
            node.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode',
                                     self._store('safety_mode'), LATCHED)
        except ImportError:
            pass

    def _store(self, key):
        def cb(msg):
            self.latest[key] = (time.monotonic(), msg)
        return cb

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def check(self, ok, text, required=True):
        self.results.append((bool(ok), required, text))
        tag = '[OK]  ' if ok else ('[FAIL]' if required else '[WARN]')
        self.node.get_logger().info(f'{tag} {text}') if ok else (
            self.node.get_logger().error(f'{tag} {text}') if required else
            self.node.get_logger().warn(f'{tag} {text}'))
        return ok

    def run(self, execution=False, wait=3.0):
        self.spin_for(wait)
        # 1. joint states
        js = self.latest.get('joint_states')
        fresh = js is not None and time.monotonic() - js[0] < 1.0
        names_ok = js is not None and set(JOINTS).issubset(js[1].name)
        self.check(fresh and names_ok, '/joint_states fresh with all six UR5e joints'
                   if fresh and names_ok else '/joint_states missing/stale: is driver.launch.py running '
                   'with ROS_DOMAIN_ID=57 in this terminal too?')
        # 2. controllers
        states = self.list_controllers()
        if states is None:
            self.check(False, '/controller_manager/list_controllers not available (driver not running?)')
        else:
            self.check(states.get('joint_state_broadcaster') == 'active', 'joint_state_broadcaster active')
            ctl = states.get(self.controller)
            if execution:
                self.check(ctl == 'active', f'{self.controller} active (state: {ctl}); '
                           'start driver.launch.py with enable_execution:=true')
            else:
                self.check(ctl is not None, f'{self.controller} loaded (state: {ctl}; inactive is fine for planning)')
            other = [n for n in ('joint_trajectory_controller', 'forward_position_controller',
                                 'forward_velocity_controller') if states.get(n) == 'active']
            self.check(not other, 'no competing motion controller active' if not other
                       else 'competing controllers active: ' + ', '.join(other))
        # 3. MoveIt
        self.check(self.action_exists('/move_action'), 'MoveIt /move_action available (moveit_real.launch.py)')
        self.check(self.service_exists('/compute_cartesian_path'), 'MoveIt /compute_cartesian_path available')
        if execution:
            self.check(self.action_exists('/execute_trajectory'), 'MoveIt /execute_trajectory available')
        # 4. robot program and modes
        prog = self.latest.get('program')
        self.check(prog is not None and prog[1].data,
                   'robot_program_running == true' if prog is not None and prog[1].data else
                   'robot_program_running is not true: load the External Control program on the '
                   'pendant (Host IP = this PC, port 50002) and press Play AFTER the driver started',
                   required=execution)
        rm = self.latest.get('robot_mode')
        if rm is not None:
            self.check(rm[1].mode == 7, f'robot mode {rm[1].mode} (7 = RUNNING)', required=execution)
        sm = self.latest.get('safety_mode')
        if sm is not None:
            self.check(sm[1].mode == 1, f'safety mode {sm[1].mode} (1 = NORMAL)', required=execution)
        sp = self.latest.get('speed')
        if sp is not None:
            val = sp[1].data
            self.check(val <= 30.0, f'speed scaling {val:.0f}% (start at 10-20% on the pendant slider)',
                       required=False)
        failed = [t for ok, req, t in self.results if req and not ok]
        return not failed

    def list_controllers(self):
        try:
            from controller_manager_msgs.srv import ListControllers
        except ImportError:
            return None
        cli = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return None
        return {c.name: c.state for c in fut.result().controller}

    def action_exists(self, name):
        topic = name + '/_action/status'
        for _ in range(20):
            if any(t == topic for t, _types in self.node.get_topic_names_and_types()):
                return True
            self.spin_for(0.1)
        return False

    def service_exists(self, name):
        for _ in range(20):
            if any(s == name for s, _types in self.node.get_service_names_and_types()):
                return True
            self.spin_for(0.1)
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--execution', action='store_true', help='Also require readiness for physical execution')
    ap.add_argument('--controller', default='scaled_joint_trajectory_controller')
    args = ap.parse_args()
    rclpy.init()
    node = Node('mas248_validate_stack')
    try:
        ok = StackChecker(node, args.controller).run(execution=args.execution)
        node.get_logger().info('Stack check passed.' if ok else 'Stack check FAILED: fix the items above first.')
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
