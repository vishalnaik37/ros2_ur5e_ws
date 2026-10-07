#!/usr/bin/env python3
"""Interactive end-effector pose commander for the UR5e.

Asks the user for a goal position and orientation on the terminal, solves IK for
it with move_group (/compute_ik), plans and executes a motion to that joint goal
(MoveGroup action), and reports the pose the robot actually reached.

Pipeline:
    this script --/compute_ik, /move_action--> move_group --FollowJointTrajectory--> ur_robot_driver
    (ros2_control: mock hardware, Isaac Sim, or the real robot)
"""

import argparse
import math
import re
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

# Planning groups tried in order when --group is not given:
# ur_moveit_config (UR5e, Stage 1) and ur5_moveit_config (Isaac Sim, Stage 2).
PREFERRED_GROUPS = ("ur_manipulator", "arm", "manipulator")

UR5E_REACH = 0.85  # m, nominal reach of the UR5e from the shoulder axis

# The robot stands on a floor/table at z = 0 of base_link. MoveIt only knows the robot itself, so
# without this box it happily plans through the mounting surface (and in Isaac Sim the arm then
# hits the simulated ground). The top sits 1 cm below the base so the base itself is not in collision.
FLOOR_SIZE = (2.0, 2.0, 0.02)  # m
FLOOR_TOP_Z = -0.01  # m, in base_link

PLAN_ATTEMPTS = 3  # OMPL is sampling-based: retry a failed plan before giving up
JOINT_TOL = 1e-3  # rad, tolerance of the joint goal computed by IK
# Elbow-up, tool-down pose used to seed IK (shoulder_pan is set per seed).
READY_SEED = {"shoulder_lift_joint": -math.pi / 2, "elbow_joint": math.pi / 2,
              "wrist_1_joint": -math.pi / 2, "wrist_2_joint": -math.pi / 2, "wrist_3_joint": 0.0}
# Joints of the arm group; anything else in /joint_states (e.g. the gripper) is left out of the goal.
ARM_JOINTS = ("shoulder_pan_joint",) + tuple(READY_SEED)

# Readable names for the MoveIt error codes users are likely to see.
ERROR_NAMES = {
    v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)
}

HELP = """
Enter a goal as one of:
  x y z                         position in metres, keep the current orientation
  x y z roll pitch yaw          position in metres, orientation in DEGREES (RPY, fixed axes X-Y-Z)
  x y z qx qy qz qw             position in metres, orientation as a quaternion
Commands:
  current   print the current end-effector pose
  help      show this message
  q         quit
Example (tool pointing straight down):  0.4 0.2 0.4 180 0 0
"""


def quaternion_from_rpy(roll, pitch, yaw):
    """Quaternion (x, y, z, w) for fixed-axis roll/pitch/yaw in radians (same as tf2 setRPY)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(x, y, z, w):
    """Inverse of quaternion_from_rpy, in radians."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def parse_goal(text):
    """Parse 3 (xyz), 6 (xyz + rpy deg) or 7 (xyz + quaternion) numbers.

    Returns (xyz, quat); quat is None when only a position was given.
    """
    try:
        values = [float(v) for v in text.replace(",", " ").split()]
    except ValueError:
        raise ValueError("all values must be numbers")
    if len(values) == 3:
        quat = None
    elif len(values) == 6:
        rpy = [math.radians(a) for a in values[3:]]
        quat = quaternion_from_rpy(*rpy)
    elif len(values) == 7:
        quat = values[3:]
        norm = math.sqrt(sum(q * q for q in quat))
        if norm < 1e-6:
            raise ValueError("quaternion must not be all zeros")
        quat = [q / norm for q in quat]
    else:
        raise ValueError(f"expected 3, 6 or 7 numbers, got {len(values)}")
    return values[:3], quat


class PoseCommander(Node):
    def __init__(self, args):
        super().__init__("ur5e_pose_commander")
        self.args = args
        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        self.current_joints = {}
        self.create_subscription(JointState, "joint_states", self.on_joint_states, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ---------------------------------------------------------------- helpers
    def wait_for_move_group(self, timeout):
        self.get_logger().info("Waiting for move_group action server '/move_action'...")
        if not self.move_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                "move_group is not running. Start the UR driver and ur_moveit_config first "
                "(see README).")
            return False
        return True

    def planning_groups(self, timeout=5.0):
        """Planning group names from move_group's SRDF, or [] if they can't be read."""
        client = self.create_client(GetParameters, "/move_group/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return []
            future = client.call_async(GetParameters.Request(names=["robot_description_semantic"]))
            if not wait(future, timeout) or not future.result().values:
                return []
            srdf = future.result().values[0].string_value
            return re.findall(r'<group\s+name="([^"]+)"', srdf)
        finally:
            self.destroy_client(client)

    def add_floor(self, timeout=5.0):
        """Add the floor below the robot to MoveIt's planning scene. Returns True on success."""
        client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return False
            floor = CollisionObject(id="floor", operation=CollisionObject.ADD)
            floor.header.frame_id = self.args.frame
            floor.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(FLOOR_SIZE))]
            pose = Pose()
            pose.position.z = FLOOR_TOP_Z - FLOOR_SIZE[2] / 2
            pose.orientation.w = 1.0
            floor.primitive_poses = [pose]
            scene = PlanningScene(is_diff=True)
            scene.world.collision_objects = [floor]
            scene.robot_state.is_diff = True
            future = client.call_async(ApplyPlanningScene.Request(scene=scene))
            return wait(future, timeout) and future.result().success
        finally:
            self.destroy_client(client)

    def select_group(self):
        """Use --group if valid, otherwise pick a known arm group from move_group. False on error."""
        groups = self.planning_groups()
        if not groups:
            if self.args.group is None:
                self.args.group = PREFERRED_GROUPS[0]
            self.get_logger().warn(
                f"Could not read planning groups from move_group, using '{self.args.group}'")
            return True
        if self.args.group is None:
            self.args.group = next((g for g in PREFERRED_GROUPS if g in groups), groups[0])
        elif self.args.group not in groups:
            self.get_logger().error(
                f"Planning group '{self.args.group}' does not exist. Available: {', '.join(groups)}")
            return False
        return True

    def current_pose(self, timeout=2.0):
        """End-effector pose (xyz, quat) in the goal frame, from TF. None if unavailable."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.args.frame, self.args.ee_link, Time(), timeout=Duration(seconds=timeout))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Cannot read current pose: {e}")
            return None
        p, q = t.transform.translation, t.transform.rotation
        return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

    @staticmethod
    def print_pose(label, pose):
        """Print a (xyz, quat) pose; does nothing for None."""
        if pose is None:
            return
        p, q = pose
        rpy = [math.degrees(a) for a in rpy_from_quaternion(*q)]
        print(f"{label}: position [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m | "
              f"rpy [{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}] deg | "
              f"quat [{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]")

    # ------------------------------------------------------------- the goal
    def goal_from_text(self, text):
        """Parse user input; a position-only goal keeps the current orientation."""
        xyz, quat = parse_goal(text)
        if quat is None:
            pose = self.current_pose()
            if pose is None:
                raise ValueError("current orientation unknown (no TF), give the orientation too")
            quat = pose[1]
        return xyz, quat

    def target_pose(self, xyz, quat):
        target = PoseStamped()
        target.header.frame_id = self.args.frame
        target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
        target.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        return target

    def ik_seeds(self, current):
        """IK seeds: the current joints, then elbow-up 'ready' poses at several base rotations.

        The UR joints can turn +-360 deg, so a pose has many IK solutions, and from a singular
        start (e.g. the arm pointing straight up) KDL falls back to random restarts that can
        return a solution the arm cannot reach without passing through itself. Seeding from
        well-conditioned poses and keeping the solution nearest to the current state avoids that.
        """
        seeds = [dict(current)]
        pan = current.get("shoulder_pan_joint", 0.0)
        for dpan in (0.0, math.pi / 2, -math.pi / 2, math.pi):
            seed = dict(current)
            seed.update(READY_SEED)
            seed["shoulder_pan_joint"] = pan + dpan
            seeds.append({n: p for n, p in seed.items() if n in current})
        return seeds

    def solve_ik(self, xyz, quat, timeout=0.5):
        """Collision-free joint positions for the pose that are nearest to the current joints.

        Returns {joint name: position} or None if IK finds no solution.
        """
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            return None
        current = self.current_joints
        seeds = self.ik_seeds(current) if current else [None]
        best, best_cost = None, math.inf
        for seed in seeds:
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.args.group
            req.ik_request.ik_link_name = self.args.ee_link
            req.ik_request.pose_stamped = self.target_pose(xyz, quat)
            req.ik_request.avoid_collisions = True
            req.ik_request.timeout = Duration(seconds=timeout).to_msg()
            if seed is not None:
                req.ik_request.robot_state.joint_state.name = list(seed)
                req.ik_request.robot_state.joint_state.position = list(seed.values())
            future = self.ik_client.call_async(req)
            if not wait(future, timeout + 5.0):
                continue
            res = future.result()
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            js = res.solution.joint_state
            joints = {n: p for n, p in zip(js.name, js.position) if n in ARM_JOINTS}
            cost = sum(abs(p - current[n]) for n, p in joints.items()) if current else 0.0
            if cost < best_cost:
                best, best_cost = joints, cost
        return best

    def on_joint_states(self, msg):
        self.current_joints = dict(zip(msg.name, msg.position))

    def joint_constraints(self, joints):
        """Goal constraints: every joint at the IK solution, within JOINT_TOL."""
        return Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=pos, tolerance_above=JOINT_TOL,
                            tolerance_below=JOINT_TOL, weight=1.0)
            for name, pos in joints.items()])

    def pose_constraints(self, xyz, quat):
        """Goal constraints: `xyz` within a small sphere and `quat` within a small tolerance."""
        a = self.args
        target = self.target_pose(xyz, quat)
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[a.pos_tol])
        position = PositionConstraint()
        position.header = target.header
        position.link_name = a.ee_link
        position.constraint_region = BoundingVolume(
            primitives=[sphere], primitive_poses=[target.pose])
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header = target.header
        orientation.link_name = a.ee_link
        orientation.orientation = target.pose.orientation
        orientation.absolute_x_axis_tolerance = a.rot_tol
        orientation.absolute_y_axis_tolerance = a.rot_tol
        orientation.absolute_z_axis_tolerance = a.rot_tol
        orientation.weight = 1.0
        return Constraints(position_constraints=[position], orientation_constraints=[orientation])

    def build_goal(self, constraints):
        """MoveGroup goal: plan from the current state to `constraints` (and execute it)."""
        a = self.args
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = a.group
        req.num_planning_attempts = 10
        req.allowed_planning_time = a.planning_time
        req.max_velocity_scaling_factor = a.velocity
        req.max_acceleration_scaling_factor = a.acceleration
        req.start_state.is_diff = True  # start from the robot's current state
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = a.plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        return goal

    def move_to(self, xyz, quat):
        """Plan (and execute) a motion to the pose. Returns True on success.

        The pose is converted to a joint goal with IK first; planning to exact joint values is
        much more reliable than letting OMPL sample a pose goal with tight tolerances. If IK finds
        no solution, the pose constraints are sent instead. Failed plans are retried, since OMPL
        is randomised and a second attempt often succeeds.
        """
        dist = math.sqrt(sum(c * c for c in xyz))
        if dist > UR5E_REACH:
            print(f"Warning: goal is {dist:.2f} m from the base, beyond the UR5e reach "
                  f"(~{UR5E_REACH} m). Planning will probably fail.")

        joints = self.solve_ik(xyz, quat)
        if joints is not None:
            print("IK: " + ", ".join(f"{n.replace('_joint', '')} {math.degrees(p):.1f}"
                                     for n, p in joints.items()) + " deg")
            constraints = self.joint_constraints(joints)
        else:
            print("IK found no collision-free solution, planning to the pose directly...")
            constraints = self.pose_constraints(xyz, quat)

        for attempt in range(1, PLAN_ATTEMPTS + 1):
            code = self.send_goal(self.build_goal(constraints))
            if code == MoveItErrorCodes.SUCCESS:
                print("Plan computed (not executed, --plan-only)." if self.args.plan_only
                      else "Goal reached.")
                return True
            if code is None:
                return False
            print(f"Failed: {ERROR_NAMES.get(code, code)}")
            # Only planning failures are worth retrying; execution or input errors are not.
            if code not in (MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.FAILURE,
                            MoveItErrorCodes.TIMED_OUT) or attempt == PLAN_ATTEMPTS:
                return False
            print(f"Retrying ({attempt + 1}/{PLAN_ATTEMPTS})...")
        return False

    def send_goal(self, goal):
        """Send a MoveGroup goal and wait for the result. MoveIt error code, or None on no answer."""
        send_future = self.move_client.send_goal_async(goal)
        if not wait(send_future, 10.0):
            print("No response from move_group.")
            return None
        handle = send_future.result()
        if not handle.accepted:
            print("Goal rejected by move_group.")
            return None
        print("Goal accepted, planning..." if self.args.plan_only
              else "Goal accepted, planning and executing...")

        result_future = handle.get_result_async()
        if not wait(result_future, self.args.planning_time + 120.0):
            print("Timed out waiting for the motion to finish; cancelling.")
            handle.cancel_goal_async()
            return None
        return result_future.result().result.error_code.val


def wait(future, timeout):
    """Block the calling thread until `future` is done (the executor runs in another thread)."""
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end or not rclpy.ok():
            return False
        time.sleep(0.02)
    return True


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--goal", nargs="+", type=float, metavar="V",
                   help="non-interactive: 'x y z', 'x y z roll pitch yaw' (deg) or 'x y z qx qy qz qw'")
    p.add_argument("--group", default=None,
                   help="MoveIt planning group (default: auto, 'ur_manipulator' or 'arm')")
    p.add_argument("--ee-link", default="tool0", help="end-effector link to place at the goal")
    p.add_argument("--frame", default="base_link", help="frame the goal is expressed in")
    p.add_argument("--velocity", type=float, default=0.2, help="velocity scaling 0-1")
    p.add_argument("--acceleration", type=float, default=0.2, help="acceleration scaling 0-1")
    p.add_argument("--planning-time", type=float, default=5.0, help="seconds per planning attempt")
    p.add_argument("--pos-tol", type=float, default=0.001, help="position tolerance radius [m]")
    p.add_argument("--rot-tol", type=float, default=0.01, help="orientation tolerance [rad]")
    p.add_argument("--plan-only", action="store_true", help="plan but do not execute")
    p.add_argument("--no-floor", action="store_true",
                   help="do not add the floor (z = 0 of --frame) to the planning scene")
    return p.parse_args(argv)


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:]
    args = parse_args(argv)

    rclpy.init()
    node = PoseCommander(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    rc = 0
    try:
        if not node.wait_for_move_group(timeout=15.0) or not node.select_group():
            rc = 1
        elif not args.no_floor and not node.add_floor():
            print("Could not add the floor to the planning scene (is move_group running?).")
            rc = 1
        elif args.goal:
            try:
                xyz, quat = node.goal_from_text(" ".join(str(v) for v in args.goal))
            except ValueError as e:
                print(f"Invalid --goal: {e}")
                return 1
            node.print_pose("Goal   ", (xyz, quat))
            rc = 0 if node.move_to(xyz, quat) else 1
            if rc == 0 and not args.plan_only:
                time.sleep(0.2)  # let TF catch up with the final joint state
                node.print_pose("Reached", node.current_pose())
        else:
            print(f"\nUR5e pose commander | group '{args.group}', end effector '{args.ee_link}', "
                  f"frame '{args.frame}'")
            print(HELP)
            node.print_pose("Current", node.current_pose())
            while rclpy.ok():
                try:
                    text = input("\ngoal> ").strip()
                except EOFError:
                    break
                if not text:
                    continue
                if text.lower() in ("q", "quit", "exit"):
                    break
                if text.lower() in ("h", "help", "?"):
                    print(HELP)
                    continue
                if text.lower() in ("c", "current"):
                    node.print_pose("Current", node.current_pose())
                    continue
                try:
                    xyz, quat = node.goal_from_text(text)
                except ValueError as e:
                    print(f"Invalid input: {e}. Type 'help' for the format.")
                    continue
                node.print_pose("Goal   ", (xyz, quat))
                if node.move_to(xyz, quat) and not args.plan_only:
                    time.sleep(0.2)  # let TF catch up with the final joint state
                    node.print_pose("Reached", node.current_pose())
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the executor thread before destroying the node, so no callback runs on a
        # half-destroyed node.
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
    return rc


if __name__ == "__main__":
    sys.exit(main())
