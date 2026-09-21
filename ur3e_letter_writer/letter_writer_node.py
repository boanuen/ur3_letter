#!/usr/bin/env python3
"""
ur3e_letter_writer / letter_writer_node
----------------------------------------
Student control node for the UR3/UR3e "write your initial" assignment.

Pipeline implemented in this node (matches the assignment's reference
flow):

    Cartesian waypoints  ->  MoveIt planning/IK  ->  joint trajectory
                                                  ->  UR3e simulation

Concretely, it talks to the move_group node that is already running
(started by ur_simulation_gz's ur_sim_moveit.launch.py) through MoveIt's
plain ROS services/actions -- no moveit_commander / moveit_py needed:

  * /plan_kinematic_path      (moveit_msgs/srv/GetMotionPlan)
        -> one full, collision-checked plan from wherever the arm
           currently is to a safe "ready" pose above the letter.
  * /compute_cartesian_path   (moveit_msgs/srv/GetCartesianPath)
        -> a single piecewise-linear Cartesian path through every
           waypoint of the letter (pen-down strokes AND the pen-up
           transitions between them).
  * /execute_trajectory       (moveit_msgs/action/ExecuteTrajectory)
        -> executes both trajectories on the simulated robot.

The letter geometry itself (which points, in which order, which segments
are pen-down vs. pen-up) comes from ur3e_letter_writer.letter_paths, which
is plain Python with no ROS dependency, so it can be -- and was -- unit
tested offline together with a numeric IK reachability check before ever
touching MoveIt.
"""

import sys
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from ur3e_letter_writer.letter_paths import build_letter_waypoints, available_letters

# Quaternion for "tool z-axis points straight down" (a 180 deg rotation
# about the tool's X axis). Computed once, offline, as:
#   axis = (1, 0, 0), angle = pi  ->  w = cos(pi/2) = 0, x = sin(pi/2) = 1
PEN_DOWN_ORIENTATION = (1.0, 0.0, 0.0, 0.0)  # (x, y, z, w)


def make_pose(x: float, y: float, z: float, quat_xyzw) -> Pose:
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    p.orientation = Quaternion(x=quat_xyzw[0], y=quat_xyzw[1],
                                z=quat_xyzw[2], w=quat_xyzw[3])
    return p


class LetterWriterNode(Node):

    def __init__(self):
        super().__init__('letter_writer_node')

        self._declare_parameters()
        p = self._read_parameters()
        self.p = p

        self.get_logger().info(
            f"Letter '{p['letter']}' | plane center=({p['x_center']:.3f}, "
            f"{p['y_center']:.3f}) size={p['width']:.3f}x{p['height']:.3f} m "
            f"| z_draw={p['z_draw']:.3f} z_lift={p['z_lift']:.3f} "
            f"(frame '{p['frame_id']}')"
        )

        self._marker_pub = self.create_publisher(
            Marker, 'letter_writer/eef_path',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        self._cartesian_cli = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._plan_cli = self.create_client(GetMotionPlan, 'plan_kinematic_path')
        self._execute_cli = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    # ------------------------------------------------------------------ #
    # Parameters
    # ------------------------------------------------------------------ #
    def _declare_parameters(self):
        self.declare_parameter('letter', 'T')
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('ee_link', 'tool0')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('x_center', 0.26)
        self.declare_parameter('y_center', 0.05)
        self.declare_parameter('width', 0.12)
        self.declare_parameter('height', 0.16)
        self.declare_parameter('z_draw', 0.06)
        self.declare_parameter('z_lift', 0.11)
        self.declare_parameter('max_velocity_scaling_factor', 0.15)
        self.declare_parameter('max_acceleration_scaling_factor', 0.15)
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('cartesian_eef_step', 0.005)
        self.declare_parameter('start_delay_sec', 8.0)

    def _read_parameters(self):
        g = self.get_parameter
        return dict(
            letter=g('letter').value,
            planning_group=g('planning_group').value,
            ee_link=g('ee_link').value,
            frame_id=g('frame_id').value,
            x_center=g('x_center').value,
            y_center=g('y_center').value,
            width=g('width').value,
            height=g('height').value,
            z_draw=g('z_draw').value,
            z_lift=g('z_lift').value,
            vel_scale=g('max_velocity_scaling_factor').value,
            acc_scale=g('max_acceleration_scaling_factor').value,
            planning_time=g('planning_time').value,
            eef_step=g('cartesian_eef_step').value,
            start_delay=g('start_delay_sec').value,
        )

    # ------------------------------------------------------------------ #
    # Startup gating: wait for move_group's services/action, then a small
    # extra grace delay so Gazebo + controllers are fully settled.
    # ------------------------------------------------------------------ #
    def wait_until_ready(self):
        self.get_logger().info(
            f"Waiting {self.p['start_delay']:.0f}s for Gazebo/controllers/"
            "MoveIt to settle, then for move_group services/action...")
        deadline_grace = time.time() + self.p['start_delay']
        while time.time() < deadline_grace:
            rclpy.spin_once(self, timeout_sec=0.5)

        while rclpy.ok() and not (
            self._cartesian_cli.service_is_ready()
            and self._plan_cli.service_is_ready()
            and self._execute_cli.server_is_ready()
        ):
            self.get_logger().info(
                'Still waiting for move_group (compute_cartesian_path / '
                'plan_kinematic_path / execute_trajectory)...',
                throttle_duration_sec=5.0,
            )
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info('move_group is ready.')

    # ------------------------------------------------------------------ #
    # Main pipeline
    # ------------------------------------------------------------------ #
    def run(self):
        p = self.p
        if p['letter'].upper() not in available_letters():
            self.get_logger().error(
                f"Letter '{p['letter']}' not supported. "
                f"Available: {available_letters()}")
            return

        waypoints = build_letter_waypoints(
            p['letter'], p['x_center'], p['y_center'],
            p['width'], p['height'], p['z_draw'], p['z_lift'],
        )
        self.get_logger().info(f'Built {len(waypoints)} Cartesian waypoints for '
                                f"letter '{p['letter']}'.")
        self._publish_path_marker(waypoints)

        # 1) Full plan+execute to a safe pose above the first stroke.
        first = waypoints[0]
        self.get_logger().info(
            f'Planning approach move to ({first.x:.3f}, {first.y:.3f}, {first.z:.3f})...')
        traj = self._plan_pose_goal(first.x, first.y, first.z)
        if traj is None:
            self.get_logger().error('Failed to plan the approach move. Aborting.')
            return
        if not self._execute(traj):
            self.get_logger().error('Failed to execute the approach move. Aborting.')
            return

        # 2) One Cartesian path through every remaining waypoint (draw +
        #    pen-up transitions between strokes).
        self.get_logger().info('Computing Cartesian path for the full letter...')
        traj, fraction = self._plan_cartesian(waypoints[1:])
        if traj is None or fraction < 0.99:
            self.get_logger().error(
                f'Cartesian path only {fraction * 100:.1f}% complete '
                '(need >=99%). Not executing -- check reachability, '
                'joint limits, or self-collision. Aborting.')
            return
        self.get_logger().info(f'Cartesian path OK ({fraction * 100:.1f}% complete). Executing...')
        if not self._execute(traj):
            self.get_logger().error('Failed to execute the letter trajectory.')
            return

        self.get_logger().info(f"Finished writing letter '{p['letter']}'.")

    # ------------------------------------------------------------------ #
    # MoveIt calls
    # ------------------------------------------------------------------ #
    def _goal_constraints_for_pose(self, x, y, z, pos_tol=1e-3, ang_tol=1e-2):
        p = self.p
        header = Header(frame_id=p['frame_id'])

        primitive = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
        bv = BoundingVolume(
            primitives=[primitive],
            primitive_poses=[make_pose(x, y, z, (0.0, 0.0, 0.0, 1.0))],
        )
        pos_constraint = PositionConstraint(
            header=header,
            link_name=p['ee_link'],
            target_point_offset=Vector3(x=0.0, y=0.0, z=0.0),
            constraint_region=bv,
            weight=1.0,
        )
        qx, qy, qz, qw = PEN_DOWN_ORIENTATION
        ori_constraint = OrientationConstraint(
            header=header,
            link_name=p['ee_link'],
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            absolute_x_axis_tolerance=ang_tol,
            absolute_y_axis_tolerance=ang_tol,
            absolute_z_axis_tolerance=ang_tol,
            weight=1.0,
        )
        return Constraints(position_constraints=[pos_constraint],
                            orientation_constraints=[ori_constraint])

    def _plan_pose_goal(self, x, y, z):
        p = self.p
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.workspace_parameters = WorkspaceParameters(
            header=Header(frame_id=p['frame_id']),
            min_corner=Vector3(x=-1.0, y=-1.0, z=-1.0),
            max_corner=Vector3(x=1.0, y=1.0, z=1.0),
        )
        mpr.start_state = RobotState()  # empty -> "use current state"
        mpr.goal_constraints = [self._goal_constraints_for_pose(x, y, z)]
        mpr.group_name = p['planning_group']
        mpr.num_planning_attempts = 5
        mpr.allowed_planning_time = p['planning_time']
        mpr.max_velocity_scaling_factor = p['vel_scale']
        mpr.max_acceleration_scaling_factor = p['acc_scale']
        req.motion_plan_request = mpr

        future = self._plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=p['planning_time'] + 5.0)
        if future.result() is None:
            self.get_logger().error('plan_kinematic_path call timed out / failed.')
            return None
        resp = future.result().motion_plan_response
        if resp.error_code.val != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS == 1
            self.get_logger().error(f'Planning failed, error code {resp.error_code.val}.')
            return None
        return resp.trajectory

    def _plan_cartesian(self, waypoints):
        p = self.p
        req = GetCartesianPath.Request()
        req.header = Header(frame_id=p['frame_id'])
        req.start_state = RobotState()  # "use current state"
        req.group_name = p['planning_group']
        req.link_name = p['ee_link']
        req.waypoints = [make_pose(w.x, w.y, w.z, PEN_DOWN_ORIENTATION) for w in waypoints]
        req.max_step = p['eef_step']
        req.jump_threshold = 0.0          # rely on collision checking, not the
        req.prismatic_jump_threshold = 0.0  # legacy jump heuristic
        req.revolute_jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = p['vel_scale']
        req.max_acceleration_scaling_factor = p['acc_scale']

        future = self._cartesian_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is None:
            self.get_logger().error('compute_cartesian_path call timed out / failed.')
            return None, 0.0
        resp = future.result()
        return resp.solution, resp.fraction

    def _execute(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        if not self._execute_cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('execute_trajectory action server not available.')
            return False
        send_future = self._execute_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('execute_trajectory goal rejected.')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error('execute_trajectory did not return a result.')
            return False
        ok = result.result.error_code.val == 1
        if not ok:
            self.get_logger().error(f'Execution failed, error code {result.result.error_code.val}.')
        return ok

    # ------------------------------------------------------------------ #
    # RViz visualization
    # ------------------------------------------------------------------ #
    def _build_markers(self, waypoints):
        """Pre-build the two markers so we can re-publish them on a timer."""
        marker = Marker()
        marker.header.frame_id = self.p['frame_id']
        marker.ns = 'letter_writer'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.004
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=w.x, y=w.y, z=w.z) for w in waypoints]

        draw_marker = Marker()
        draw_marker.header.frame_id = self.p['frame_id']
        draw_marker.ns = 'letter_writer_draw'
        draw_marker.id = 1
        draw_marker.type = Marker.LINE_LIST
        draw_marker.action = Marker.ADD
        draw_marker.scale.x = 0.006
        draw_marker.color.a = 1.0
        draw_marker.color.r = 1.0
        draw_marker.color.g = 0.1
        draw_marker.color.b = 0.1
        draw_marker.pose.orientation.w = 1.0
        pts = []
        for a, b in zip(waypoints[:-1], waypoints[1:]):
            if a.pen_down and b.pen_down:
                pts.append(Point(x=a.x, y=a.y, z=a.z))
                pts.append(Point(x=b.x, y=b.y, z=b.z))
        draw_marker.points = pts

        self._cached_markers = (marker, draw_marker)

    def _publish_path_marker(self, waypoints):
        self._build_markers(waypoints)
        self._republish_markers()

    def _republish_markers(self):
        """(Re-)publish the cached markers with a fresh timestamp."""
        if not hasattr(self, '_cached_markers'):
            return
        for m in self._cached_markers:
            m.header.stamp = self.get_clock().now().to_msg()
            self._marker_pub.publish(m)

    def start_marker_timer(self):
        """Re-publish markers every second so late-joining RViz displays
        can see them without needing to restart the node."""
        self._marker_timer = self.create_timer(1.0, self._republish_markers)


def main(args=None):
    rclpy.init(args=args)
    node = LetterWriterNode()
    try:
        node.wait_until_ready()
        node.run()
        # Keep the node alive and re-publish markers every second so the
        # user can add the Marker display in RViz at any time and still
        # see the letter. Press Ctrl+C to stop.
        node.start_marker_timer()
        node.get_logger().info(
            'Node stays alive — markers are re-published every 1s. '
            'Add /letter_writer/eef_path Marker in RViz to see the letter. '
            'Press Ctrl+C to stop.')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(
            'letter_writer_node failed:\n' + traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
