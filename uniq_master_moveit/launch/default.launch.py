from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("unrb_ra10", package_name="uniq_master_moveit").to_moveit_configs()

    # MoveIt2 demo launch
    moveit_launch = generate_demo_launch(moveit_config)

    # Node for publishing to follow_joint_trajectory
    follow_trajectory_publisher_node = Node(
        package='uniq_master_moveit',
        executable='follow_trajectory_publisher.py',
        name='follow_trajectory_publisher',
        output='screen',
    )
    rviz_marker_node = Node(
        package='uniq_master',
        executable='marker',
        name='rviz_marker',
        output='screen',
    )

    # Create the LaunchDescription and add actions
    ld = LaunchDescription()
    ld.add_action(moveit_launch)
    ld.add_action(follow_trajectory_publisher_node)
    ld.add_action(rviz_marker_node)

    return ld
