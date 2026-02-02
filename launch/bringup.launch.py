import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("sixdof_rviz_control")

    urdf_path = os.path.join(pkg_share, "urdf", "sixdof_arm.urdf")
    controllers_path = os.path.join(pkg_share, "config", "controllers.yaml")
    rviz_path = os.path.join(pkg_share, "rviz", "view.rviz")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controllers_path
        ],
    )

    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

  
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # RViz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        output="screen",
    )

  
    traj_planner = Node(
        package="sixdof_rviz_control",
        executable="joint_quintic_traj_node",
        output="screen",
        parameters=[{
            "joint_names": ["joint1","joint2","joint3","joint4","joint5","joint6"],
            "joint_states_topic": "/joint_states",
            "target_topic": "/target_joint_angles",
            "traj_pub_topic": "/joint_trajectory_controller/joint_trajectory",
            "duration_sec": 2.0,
            "dt_sec": 0.02,
            "include_vel_acc": True
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        spawn_jsb,
        spawn_jtc,
        rviz2,
        traj_planner,
    ])
