#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class XArm5Commander(Node):
    def __init__(self):
        super().__init__('xarm5_commander')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/xarm5_traj_controller/joint_trajectory',
            10
        )

    def send_pose(self, positions, duration_sec=3):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        msg.points.append(point)

        wait_count = 0
        while self.pub.get_subscription_count() == 0:
            self.get_logger().info('Очікування підключення контролера до /xarm5_traj_controller/joint_trajectory...')
            time.sleep(0.5)
            wait_count += 1
            if wait_count > 20:
                self.get_logger().error('Не вдалося дочекатися підписника контролера!')
                return

        self.pub.publish(msg)
        self.get_logger().info(f'Відправлено цільову позицію суглобів: {positions}')


def main():
    rclpy.init()
    node = XArm5Commander()
    try:
        # Поза 1
        node.get_logger().info('Рух до позиції 1...')
        node.send_pose([0.5, -0.4, -0.6, 0.3, 0.4], duration_sec=3)
        time.sleep(4.0)

        # Повернення у вихідну позицію (нули)
        node.get_logger().info('Повернення у нульову позицію...')
        node.send_pose([0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=3)
        time.sleep(3.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
