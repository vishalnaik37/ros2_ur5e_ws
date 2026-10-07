"""Stage 2: UR5e simulated in Isaac Sim + MoveIt + RViz.

Start Isaac Sim (isaac/ur5e_isaac_scene.py) and this launch file, in either order. It starts
  - robot_state_publisher with the UR5e from ur_description (same model as Stage 1)
  - ros2_control_node with topic_based_ros2_control as the hardware: position commands go to
    Isaac Sim on /isaac_joint_commands, joint states come back on /isaac_joint_states
  - joint_state_broadcaster + joint_trajectory_controller, and with use_gripper:=true (default) the
    Robotiq 2F-85 on tool0 with its robotiq_gripper_controller (start the scene with the gripper too)
  - move_group + RViz with ur_moveit_config (same SRDF, kinematics and planners as Stage 1)
All nodes use the simulation clock that Isaac Sim publishes on /clock. The controller manager only runs
once that clock ticks, so the controllers are spawned after the first /clock message; spawned earlier,
they time out, joint_state_broadcaster never starts and RViz shows the arm at all-zero joints (flat).

Then run the pose commander in a separate terminal, exactly as in Stage 1:
  ros2 run ur5e_pose_commander pose_commander
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory("ur5e_pose_commander")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_gripper = LaunchConfiguration("use_gripper")
    use_sim_time = {"use_sim_time": True}

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            os.path.join(pkg, "urdf", "ur5e_isaac.urdf.xacro"), {"ur_type": "ur5e", "use_gripper": use_gripper})
        .robot_description_semantic(
            Path(pkg) / "srdf" / "ur5e.srdf.xacro", {"name": "ur", "use_gripper": use_gripper})
        .trajectory_execution(os.path.join(pkg, "config", "ur5e_isaac_moveit_controllers.yaml"))
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[os.path.join(pkg, "config", "ur5e_isaac_controllers.yaml"), use_sim_time],
        remappings=[("~/robot_description", "/robot_description")],
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for controller in ("joint_state_broadcaster", "joint_trajectory_controller")
    ]
    spawners.append(Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(use_gripper),
    ))

    # Exits on the first /clock message from Isaac Sim (the type is given, so it waits for the topic).
    wait_for_clock = ExecuteProcess(
        cmd=["ros2", "topic", "echo", "--once", "/clock", "rosgraph_msgs/msg/Clock"],
        output="log",
    )
    start_controllers = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_clock,
        on_exit=[LogInfo(msg="Got /clock from Isaac Sim, spawning the controllers"), *spawners],
    ))

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
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
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Start RViz"),
        DeclareLaunchArgument(
            "use_gripper", default_value="true",
            description="Robotiq 2F-85 on tool0 (must match the Isaac scene's --no-gripper)"),
        robot_state_publisher,
        ros2_control_node,
        LogInfo(msg="Waiting for /clock from Isaac Sim (start isaac/ur5e_isaac_scene.py)"),
        wait_for_clock,
        start_controllers,
        move_group,
        rviz,
    ])
