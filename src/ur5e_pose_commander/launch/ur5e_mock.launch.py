"""Stage 1: UR5e with mock (fake) hardware + MoveIt + RViz, using the official UR packages.

Starts
  - ur_robot_driver (ur_control.launch.py) with use_mock_hardware:=true and, with
    use_gripper:=true (default), a Robotiq 2F-85 on tool0 (urdf/ur5e_robotiq.urdf.xacro)
    plus its robotiq_gripper_controller: ros2_control with a
    mock hardware interface that mirrors commands to states, so no robot or simulator is needed.
  - move_group + RViz with the MotionPlanning panel, set up like ur_moveit_config's ur_moveit.launch.py
    (kinematics, planners, controllers) but with this package's srdf/ur5e.srdf.xacro, which adds the
    gripper's planning group and collision exclusions. The stock UR SRDF knows nothing about the
    gripper, so MoveIt would see its links touching each other and reject every start state.

Then run the pose commander in a separate terminal (it reads goals from stdin):
  ros2 run ur5e_pose_commander pose_commander
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory("ur5e_pose_commander")
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_gripper = LaunchConfiguration("use_gripper")

    # arm + Robotiq 2F-85, or the stock arm-only description
    description_file = PythonExpression([
        "'", PathJoinSubstitution([FindPackageShare("ur5e_pose_commander"), "urdf", "ur5e_robotiq.urdf.xacro"]),
        "' if '", use_gripper, "' == 'true' else '",
        PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "urdf", "ur.urdf.xacro"]), "'",
    ])

    # Scoped, so that the driver's launch_rviz:=false does not leak into the MoveIt include below
    # (included launch files otherwise share and overwrite each other's launch arguments).
    driver = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": "0.0.0.0",  # unused with mock hardware, but required by the launch file
            "use_mock_hardware": "true",
            "launch_rviz": "false",  # RViz comes from MoveIt below
            "description_file": description_file,
        }.items(),
    )])

    # No robot_description here: like ur_moveit.launch.py, move_group and RViz take the URDF that the
    # driver publishes on /robot_description, so MoveIt always plans with the model that is running.
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            Path(pkg) / "srdf" / "ur5e.srdf.xacro", {"name": ur_type, "use_gripper": use_gripper})
        .to_moveit_configs()
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"publish_robot_description_semantic": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", os.path.join(
            get_package_share_directory("ur_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    moveit = RegisterEventHandler(OnProcessExit(
        target_action=wait_robot_description, on_exit=[move_group, rviz]))

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(use_gripper),
        arguments=[
            "robotiq_gripper_controller",
            "--param-file",
            PathJoinSubstitution([FindPackageShare("ur5e_pose_commander"), "config", "ur5e_gripper_controllers.yaml"]),
            "--controller-manager-timeout", "30",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e", description="UR robot model"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Start RViz"),
        DeclareLaunchArgument(
            "use_gripper", default_value="true", description="Mount the Robotiq 2F-85 gripper on tool0"),
        driver,
        gripper_controller_spawner,
        wait_robot_description,
        moveit,
    ])
