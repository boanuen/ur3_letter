"""
Launch file for the UR3/UR3e "letter writer" assignment.

    Launch file
       -> UR3e Gazebo simulation + ros2_control  (ur_simulation_gz)
       -> MoveIt 2 (move_group + RViz)           (ur_moveit_config)
       -> Student control node                   (this package)
              -> Cartesian waypoints
              -> MoveIt planning / IK
              -> joint trajectory
              -> UR3e simulation

Everything from Universal Robots' own packages is reused as-is via
IncludeLaunchDescription; only the student control node is ours.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    def arg(name, default, description):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=str(default), description=description))

    # --- UR / simulation arguments (forwarded to ur_simulation_gz) ---
    arg('ur_type', 'ur3e', 'Type of UR robot (ur3, ur3e, ...).')
    arg('launch_rviz', 'true', 'Launch RViz with the MoveIt plugin.')
    arg('gazebo_gui', 'true', 'Start Gazebo with its GUI.')
    arg('safety_limits', 'true', 'Enable the safety limits controller.')
    # Our own controllers.yaml: identical to ur_simulation_gz's, except the
    # per-joint trajectory (path) tolerance is relaxed -- see
    # config/ur_controllers.yaml for why. Point these back at
    # 'ur_simulation_gz' / 'ur_controllers.yaml' to use the stock UR config.
    arg('runtime_config_package', 'ur3e_letter_writer',
        'Package whose config/<controllers_file> is used for ros2_control.')
    arg('controllers_file', 'ur_controllers.yaml',
        'YAML file (in runtime_config_package/config) with the controllers configuration.')

    # --- Letter-writer arguments (forwarded to our control node) ---
    arg('letter', 'T', 'Letter to draw (see ur3e_letter_writer/letter_paths.py).')
    arg('planning_group', 'ur_manipulator', 'MoveIt planning group name.')
    arg('ee_link', 'tool0', 'End-effector / pen-tip link name.')
    arg('frame_id', 'base_link', 'Frame the letter waypoints are expressed in.')
    arg('x_center', 0.26, 'Letter plane center, X (m), in frame_id.')
    arg('y_center', 0.05, 'Letter plane bottom edge, Y (m), in frame_id.')
    arg('width', 0.12, 'Letter width (m).')
    arg('height', 0.16, 'Letter height (m).')
    arg('z_draw', 0.06, 'Pen-down Z height (m).')
    arg('z_lift', 0.11, 'Pen-up / retreat Z height (m).')
    arg('max_velocity_scaling_factor', 0.15, 'MoveIt velocity scaling (0-1).')
    arg('max_acceleration_scaling_factor', 0.15, 'MoveIt acceleration scaling (0-1).')
    arg('start_delay_sec', 8.0, 'Grace period (s) before the control node starts.')
    arg('start_control_node', 'true', 'Whether to launch the student control node at all.')

    ur_type = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    safety_limits = LaunchConfiguration('safety_limits')

    ur_sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ur_simulation_gz'), '/launch', '/ur_sim_moveit.launch.py']
        ),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': launch_rviz,
            'safety_limits': safety_limits,
            'runtime_config_package': LaunchConfiguration('runtime_config_package'),
            'controllers_file': LaunchConfiguration('controllers_file'),
        }.items(),
    )
    # NOTE: `gazebo_gui` is not one of ur_sim_moveit.launch.py's own
    # declared arguments -- it belongs to ur_sim_control.launch.py, two
    # levels down. ROS 2 launch configurations share one flat namespace
    # across an include tree, so declaring `gazebo_gui` here (above) is
    # enough: the nested DeclareLaunchArgument('gazebo_gui', ...) will see
    # it already set and use our value instead of its own default.

    control_node = Node(
        package='ur3e_letter_writer',
        executable='letter_writer_node',
        name='letter_writer_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_control_node')),
        parameters=[{
            'letter': LaunchConfiguration('letter'),
            'planning_group': LaunchConfiguration('planning_group'),
            'ee_link': LaunchConfiguration('ee_link'),
            'frame_id': LaunchConfiguration('frame_id'),
            'x_center': LaunchConfiguration('x_center'),
            'y_center': LaunchConfiguration('y_center'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'z_draw': LaunchConfiguration('z_draw'),
            'z_lift': LaunchConfiguration('z_lift'),
            'max_velocity_scaling_factor': LaunchConfiguration('max_velocity_scaling_factor'),
            'max_acceleration_scaling_factor': LaunchConfiguration('max_acceleration_scaling_factor'),
            'start_delay_sec': LaunchConfiguration('start_delay_sec'),
        }],
    )

    # Small extra delay at the launch-description level too, so the node
    # process itself (and its parameter loading) doesn't race the very
    # first Gazebo/controller spawn events.
    delayed_control_node = TimerAction(period=2.0, actions=[control_node])

    return LaunchDescription(declared_arguments + [ur_sim_moveit, delayed_control_node])
