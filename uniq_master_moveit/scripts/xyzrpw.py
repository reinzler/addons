#!/usr/bin/env python3
#
# # Generic ROS libraries
# import rclpy
# from rclpy.logging import get_logger
# from geometry_msgs.msg import PoseStamped
#
# # MoveIt python library
# from moveit.planning import (
#     MoveItPy,
# )
#
#
# def main():
#
#     # Initialize rclpy and ROS logger
#     rclpy.init()
#     logger = get_logger("moveit_py.pose_goal")
#
#     # Instantiate MoveItPy and get planning component
#     ra_10 = MoveItPy(node_name="moveit_py")
#     ra_10_arm = ra_10.get_planning_component("ra_10")
#
#     # set plan start state to current state
#     ra_10_arm.set_start_state_to_current_state()
#
#     # Create PoseStamped message that will hold the target Pose
#     pose_goal = PoseStamped()
#     pose_goal.header.frame_id = "base_link"
#
#     # Describe the target pose for the end-effector
#     pose_goal.pose.orientation.w = 1.0
#     pose_goal.pose.position.x = 0.28
#     pose_goal.pose.position.y = -0.2
#     pose_goal.pose.position.z = 0.5
#
#     # Set the target pose
#     ra_10_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="ra_10_link8")
#
#     # Create a plan to the target pose
#     plan_result = ra_10_arm.plan()
#
#     # If the plan is successful, get the trajectory and execute the plan
#     if plan_result:
#         robot_trajectory = plan_result.trajectory
#         ra_10.execute(robot_trajectory, blocking=True, controllers=[])
#     else:
#         logger.error("Planning failed")
#
#     rclpy.shutdown()
#
#
# if __name__ == "__main__":
#     main()
#

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit2 import MoveIt2
from tf_transformations import quaternion_from_euler


class MoveIt2Node(Node):
    def __init__(self):
        super().__init__('moveit2_node')
        self.moveit2 = MoveIt2(self)

    def move_to_xyzrpw(self, x, y, z, roll, pitch, yaw):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]

        self.moveit2.set_pose_target(pose_goal)
        success = self.moveit2.go(wait=True)
        self.moveit2.stop()
        self.moveit2.clear_pose_targets()
        return success


def main(args=None):
    rclpy.init(args=args)
    node = MoveIt2Node()

    # Пример координат XYZRPW
    x = 0.4
    y = 0.2
    z = 0.3
    roll = 0.0
    pitch = 1.57
    yaw = 0.0

    if node.move_to_xyzrpw(x, y, z, roll, pitch, yaw):
        node.get_logger().info("Movement successful")
    else:
        node.get_logger().info("Movement failed")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
