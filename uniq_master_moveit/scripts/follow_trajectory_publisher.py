#!/usr/bin/env python3
'''
ros2 topic pub /ra_10_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {stamp: now,
frame_id: "base_link"}, joint_names: ["Revolute 1", "Revolute 2", "Revolute 3", "Revolute 4", "Revolute 5",
"Revolute 6"], points: [{positions: [1.1, 0.0, 0.0, 0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 1, nanosec: 0}}]}'
'''

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time import sleep
from colorama import Fore
from interfaces.msg import InverseKinematics, InverseKinematicsOutput


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('follow_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/ra_10_controller/joint_trajectory', 10)
        self.desire_pose_publisher = self.create_publisher(InverseKinematics, "/desired_pose", 10)
        # Задержка перед началом публикации, чтобы дать MoveIt2 время на загрузку
        self.get_logger().info(f'{Fore.CYAN}Waiting for MoveIt2 to load...{Fore.RESET}')
        # sleep(10)  # Ожидание 10 секунд, вы можете настроить это время по своему усмотрению
        self.timer = self.create_timer(10.0, self.publish_trajectory)
        # Вызов функции публикации однократно
        # self.publish_trajectory()

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['Revolute 1', 'Revolute 2', 'Revolute 3', 'Revolute 4', 'Revolute 5', 'Revolute 6']
        point = JointTrajectoryPoint()
        point.positions = [3.07787798, 2.20991839, 1.0244009, -0.8596156, -0.65842955, 0.01331229]
        # point.positions = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 2
        msg.points = [point]
        self.publisher_.publish(msg)
        self.get_logger().info(f'{Fore.GREEN}Publishing trajectory: "%s" {msg}{Fore.RESET}')

        desire_pose_msg = InverseKinematics()
        desire_pose_msg.x = 0.5
        desire_pose_msg.y = 0.025
        desire_pose_msg.z = 0.405
        desire_pose_msg.r = -174.0
        desire_pose_msg.p = 139.0
        desire_pose_msg.w = -130.0
        self.desire_pose_publisher.publish(desire_pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
