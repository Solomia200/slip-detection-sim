#!/usr/bin/env python3
"""
Демо-скрипт для симуляції Gazebo:
1. Подвійний клік гріпера (drive_joint).
2. Поворот маніпулятора xArm 5 вліво-вправо (+20° / -20°) і повернення в нуль.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class GazeboDemo(Node):
    def __init__(self):
        super().__init__('gazebo_demo_node')
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm5_traj_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm_gripper_traj_controller/follow_joint_trajectory'
        )

    def run_demo(self):
        self.get_logger().info('Очікування підключення контролерів у Gazebo...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Контролери підключено!')

        # 1. ДЕМО ГРІПЕРА (подвійне стискання та розкриття)
        self.get_logger().info('=== 1. Демонстрація гріпера в Gazebo ===')
        gripper_goal = FollowJointTrajectory.Goal()
        gripper_goal.trajectory.joint_names = ['drive_joint']

        # Точки: стиснути (0.7) -> розкрити (0.0) -> стиснути (0.7) -> розкрити (0.0)
        p1 = JointTrajectoryPoint(positions=[0.7], time_from_start=Duration(sec=1, nanosec=500000000))
        p2 = JointTrajectoryPoint(positions=[0.0], time_from_start=Duration(sec=3, nanosec=0))
        p3 = JointTrajectoryPoint(positions=[0.7], time_from_start=Duration(sec=4, nanosec=500000000))
        p4 = JointTrajectoryPoint(positions=[0.0], time_from_start=Duration(sec=6, nanosec=0))
        gripper_goal.trajectory.points = [p1, p2, p3, p4]

        self.get_logger().info('Відправка траєкторії гріпера (2 кліки)...')
        future_gripper = self.gripper_client.send_goal_async(gripper_goal)
        rclpy.spin_until_future_complete(self, future_gripper)
        goal_handle = future_gripper.result()
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info('Гріпер відпрацював успішно!')

        time.sleep(1.0)

        # 2. ДЕМО РУКИ (поворот основи на +20° та -20°)
        self.get_logger().info('=== 2. Демонстрація повороту руки (+20° / -20°) ===')
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        # +20 deg = 0.35 rad, -20 deg = -0.35 rad
        a1 = JointTrajectoryPoint(
            positions=[0.35, 0.0, 0.0, 0.0, 0.0],
            time_from_start=Duration(sec=2, nanosec=0)
        )
        a2 = JointTrajectoryPoint(
            positions=[-0.35, 0.0, 0.0, 0.0, 0.0],
            time_from_start=Duration(sec=5, nanosec=0)
        )
        a3 = JointTrajectoryPoint(
            positions=[0.0, 0.0, 0.0, 0.0, 0.0],
            time_from_start=Duration(sec=7, nanosec=0)
        )
        arm_goal.trajectory.points = [a1, a2, a3]

        self.get_logger().info('Відправка траєкторії руки (поворот вправо -> вліво -> нуль)...')
        future_arm = self.arm_client.send_goal_async(arm_goal)
        rclpy.spin_until_future_complete(self, future_arm)
        arm_handle = future_arm.result()
        res_future_arm = arm_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future_arm)
        self.get_logger().info('Рух руки завершено успішно!')


def main():
    rclpy.init()
    demo = GazeboDemo()
    try:
        demo.run_demo()
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
