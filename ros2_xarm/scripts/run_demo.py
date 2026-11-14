#!/usr/bin/env python3
"""
Універсальний ROS 2 скрипт для xArm 5 + Gripper:
Працює з ОДНАКОВИМ кодом як у симуляції Gazebo, так і на реальному роботі.

Вибір "де запустити" визначається тим, що у вас запущено у фоні:
1. Симуляція Gazebo:
   ros2 launch xarm_gazebo _robot_beside_table_gazebo.launch.py dof:=5 robot_type:=xarm add_gripper:=true load_controller:=true
2. Реальний робот:
   ros2 launch xarm_moveit_config xarm5_moveit_realmove.launch.py robot_ip:=192.168.1.242 add_gripper:=true
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


class UnifiedXArmDemo(Node):
    def __init__(self):
        super().__init__('unified_xarm_demo')

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm5_traj_controller/follow_joint_trajectory'
        )
        self.sim_gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm_gripper_traj_controller/follow_joint_trajectory'
        )
        self.real_gripper_client = ActionClient(
            self,
            GripperCommand,
            '/xarm_gripper/gripper_action'
        )
        self._current_joints = None
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in ARM_JOINTS):
            self._current_joints = [positions[j] for j in ARM_JOINTS]

    def _send_and_wait(self, client: ActionClient, goal, label: str):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        goal_handle = fut.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'{label}: ціль відхилена контролером!')
            return None
        res_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result()

    def _read_current_arm_joints(self, timeout_sec: float = 2.0):
        deadline = time.monotonic() + timeout_sec
        while self._current_joints is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self._current_joints) if self._current_joints is not None else [0.0] * 5

    def run(self):
        self.get_logger().info('Пошук активного контролера руки (/xarm5_traj_controller)...')
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Контролер руки не знайдено! Переконайтеся, що запущено Gazebo або realmove.'
            )
            return

        is_real = self.real_gripper_client.wait_for_server(timeout_sec=1.5)
        is_gazebo = self.sim_gripper_client.wait_for_server(timeout_sec=1.5)

        if is_real and is_gazebo:
            self.get_logger().error(
                'Виявлено КОНФЛІКТ: одночасно запущено і Gazebo, і драйвер реального робота! '
                'Зупиніть попередній запуск (наприклад: docker restart -t 1 xarm5_gazebo_sim) '
                'і залиште тільки одне середовище.'
            )
            return

        env_name = "Real Hardware" if is_real else ("Gazebo Simulation" if is_gazebo else "Без гріпера")
        self.get_logger().info(f'Середовище визначено: [{env_name}]')

        # === 1. РУХ ГРІПЕРА (подвійний клік) ===
        if is_gazebo:
            self.get_logger().info('Керування гріпером (Gazebo mode)...')
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['drive_joint']
            goal.trajectory.points = [
                JointTrajectoryPoint(positions=[0.7], time_from_start=Duration(sec=1, nanosec=500000000)),
                JointTrajectoryPoint(positions=[0.0], time_from_start=Duration(sec=3, nanosec=0)),
                JointTrajectoryPoint(positions=[0.7], time_from_start=Duration(sec=4, nanosec=500000000)),
                JointTrajectoryPoint(positions=[0.0], time_from_start=Duration(sec=6, nanosec=0)),
            ]
            self._send_and_wait(self.sim_gripper_client, goal, 'Gazebo гріпер')

        elif is_real:
            self.get_logger().info('Керування гріпером (Real hardware mode)...')
            for pos in (0.7, 0.0, 0.7, 0.0):
                g = GripperCommand.Goal()
                g.command.position = pos
                self._send_and_wait(self.real_gripper_client, g, f'Real гріпер ({pos})')
                time.sleep(0.3)

        self.get_logger().info('Гріпер завершив роботу.')
        time.sleep(1.0)

        # === 2. РУХ РУКИ (+20° / -20° відносно поточної позиції joint1) ===
        base = self._read_current_arm_joints()
        j1 = base[0]
        self.get_logger().info(
            f'Керування рукою (від поточного положення {[round(x, 3) for x in base]}): '
            '+20° -> -20° -> повернення...'
        )

        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = ARM_JOINTS
        arm_goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[j1 + 0.35, *base[1:]],
                time_from_start=Duration(sec=2, nanosec=0),
            ),
            JointTrajectoryPoint(
                positions=[j1 - 0.35, *base[1:]],
                time_from_start=Duration(sec=5, nanosec=0),
            ),
            JointTrajectoryPoint(
                positions=base,
                time_from_start=Duration(sec=7, nanosec=0),
            ),
        ]

        res = self._send_and_wait(self.arm_client, arm_goal, 'Рух руки')
        if res is not None:
            if res.result.error_code == 0:
                self.get_logger().info('Успіх! Рух руки завершено.')
            else:
                self.get_logger().error(
                    f'Контролер руки повернув помилку {res.result.error_code}: {res.result.error_string}'
                )


def main():
    rclpy.init()
    node = UnifiedXArmDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
