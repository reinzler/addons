from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("unrb_ra10", package_name="uniq_master_moveit").to_moveit_configs()

    # MoveIt2 demo launch
    moveit_launch = generate_demo_launch(moveit_config)

    rviz_marker_node = Node(
        package='uniq_master',
        executable='marker',
        name='rviz_marker',
        output='screen',
    )

    rviz_overlay_node_1 = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='target_point_overlay',
        output='log',
        parameters=[
            {'string_topic': '/target_point_overlay'},
            {"fg_color": "w"},
        ]
    )

    # Create the LaunchDescription and add actions
    ld = LaunchDescription()
    ld.add_action(moveit_launch)
    ld.add_action(rviz_marker_node)
    ld.add_action(rviz_overlay_node_1)

    return ld
