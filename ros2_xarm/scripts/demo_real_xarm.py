#!/usr/bin/env python3
"""
Демо-скрипт керування реальними суглобами та гріпером UFACTORY xArm 5
IP робота за замовчуванням: 192.168.1.242
"""
import sys
import time
from xarm.wrapper import XArmAPI

ROBOT_IP = '192.168.1.242'


def main():
    print(f'Підключення до xArm 5 ({ROBOT_IP})...')
    arm = XArmAPI(ROBOT_IP)

    if not arm.connected:
        print(f'Помилка: не вдалося підключитися до {ROBOT_IP}!')
        sys.exit(1)

    print('Робот підключений. Очищення помилок та ініціалізація...')
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    arm.set_mode(0)  # позиційний режим
    arm.set_state(0)  # готовий до руху

    # Ініціалізація гріпера
    arm.set_gripper_enable(True)
    arm.set_gripper_mode(0)
    arm.set_gripper_speed(3000)

    base_angles = list(arm.angles)
    print(f'Поточні кути суглобів (град): {[round(a, 2) for a in base_angles]}')
    print(f'Поточна позиція гріпера: {arm.get_gripper_position()[1]}')

    try:
        # 1. Жест гріпером (подвійне стискання)
        print('\n--- 1. Демонстрація гріпера (подвійний клік) ---')
        for i in range(2):
            print(f'  Крок {i+1}: стискання...')
            arm.set_gripper_position(150, wait=True)
            time.sleep(0.3)
            print(f'  Крок {i+1}: розкриття...')
            arm.set_gripper_position(800, wait=True)
            time.sleep(0.3)

        # 2. Поворот основи
        print('\n--- 2. Демонстрація повороту основи (+20° / -20°) ---')
        target = list(base_angles)
        
        print('  Поворот на +20°...')
        target[0] = 20.0
        arm.set_servo_angle(angle=target, speed=25, wait=True)
        time.sleep(0.5)

        print('  Поворот на -20°...')
        target[0] = -20.0
        arm.set_servo_angle(angle=target, speed=25, wait=True)
        time.sleep(0.5)

        print('  Повернення у вихідне положення (0°)...')
        arm.set_servo_angle(angle=base_angles, speed=25, wait=True)
        time.sleep(0.5)

        print('\nВсі рухи виконано успішно!')

    finally:
        arm.disconnect()
        print('Зв\'язок коректно закрито.')


if __name__ == '__main__':
    main()
