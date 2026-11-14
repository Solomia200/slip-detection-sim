# ROS 2 Humble + Gazebo Simulation & Real Hardware for UFACTORY xArm 5

Ізольоване Docker-середовище для:
1. Симуляції маніпулятора **xArm 5** (5 DOF) у ROS 2 Humble та Gazebo Classic із `ros2_control` та гріпером.
2. Керування **реальним фізичним роботом xArm 5** (включаючи схват UFACTORY Gripper) по мережі Ethernet (IP: `192.168.1.242`).
3. Роботи через **єдиний код** (`scripts/run_demo.py`), який автоматично адаптується під симуляцію або залізо.

---

## 1. Мережеве налаштування для реального робота
* Робот має статичну IP-адресу: **`192.168.1.242`**.
* Ethernet-адаптер на ПК повинен мати статичну IP-адресу: **`192.168.1.50/24`**.
```bash
nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.50/24 ipv4.method manual
nmcli con up "Wired connection 1"
ping -c 3 192.168.1.242
```
* **Веб-інтерфейс UFACTORY Studio:** `http://192.168.1.242:18333`

---

## 2. Підготовка та запуск контейнера

```bash
# 1. Дозволити графіку з контейнера на ваш екран
xhost +local:root

# 2. Перейти в папку та підняти контейнер у фоні
cd /media/legacz/0BEF6E5B280953362/UCU/Infineon/slip-detection-sim/ros2_xarm
docker compose up -d
```

---

## 3. Режим А: Симуляція в Gazebo

### Крок 1. Запуск 3D-світу Gazebo (Термінал 1)
```bash
docker exec -it xarm5_gazebo_sim bash -c \
  "source /ros2_ws/install/setup.bash && ros2 launch xarm_gazebo _robot_beside_table_gazebo.launch.py dof:=5 robot_type:=xarm add_gripper:=true load_controller:=true"
```

### Крок 2. Запуск скрипта керування (Термінал 2)
```bash
docker exec xarm5_gazebo_sim bash -c \
  "source /ros2_ws/install/setup.bash && python3 /ros2_ws/scripts/run_demo.py"
```

---

## 4. Режим Б: Реальний фізичний робот

> [!WARNING]
> Переконайтеся, що навколо робота на столі вільно, а кнопка аварійної зупинки (E-stop) поруч!
> Також перед перемиканням із Gazebo на реального робота (або навпаки) перезапустіть контейнер, щоб не конфліктували контролери:
> `docker restart -t 1 xarm5_gazebo_sim`

### Крок 1. Запуск драйвера заліза (Термінал 1)
З вікном візуалізації RViz (потрібен `xhost +local:root`):
```bash
docker exec -it xarm5_gazebo_sim bash -c \
  "source /ros2_ws/install/setup.bash && ros2 launch xarm_moveit_config xarm5_moveit_realmove.launch.py robot_ip:=192.168.1.242 add_gripper:=true"
```
*(Або без RViz-вікна, додавши `show_rviz:=false`:)*
```bash
docker exec -it xarm5_gazebo_sim bash -c \
  "source /ros2_ws/install/setup.bash && ros2 launch xarm_moveit_config xarm5_moveit_realmove.launch.py robot_ip:=192.168.1.242 add_gripper:=true show_rviz:=false"
```

### Крок 2. Запуск того самого скрипта (Термінал 2)
```bash
docker exec xarm5_gazebo_sim bash -c \
  "source /ros2_ws/install/setup.bash && python3 /ros2_ws/scripts/run_demo.py"
```

---

## 5. Як писати та редагувати свій код

Усі скрипти лежать на вашому ПК у папці:  
`slip-detection-sim/ros2_xarm/scripts/`

Вона автоматично примонтована всередину контейнера в `/ros2_ws/scripts/`.
* Будь-які зміни в файлах на хості миттєво з'являються всередині контейнера.
* **Перезбирати або перезапускати контейнер НЕ потрібно!**

### Створення власного скрипта
Скопіюйте `run_demo.py`:
```bash
cp scripts/run_demo.py scripts/my_test.py
```
Змініть потрібні кути або таймінги, і запускайте:
```bash
docker exec xarm5_gazebo_sim bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/scripts/my_test.py"
```

---

## 6. Корисні команди діагностики (ROS 2 CLI)

Виконувати з хоста:
```bash
# Список активних контролерів (мають бути active):
docker exec xarm5_gazebo_sim bash -c "source /ros2_ws/install/setup.bash && ros2 control list_controllers"

# Поточні координати та кути суглобів:
docker exec xarm5_gazebo_sim bash -c "source /ros2_ws/install/setup.bash && ros2 topic echo /joint_states --once"

# Список усіх доступних топіків:
docker exec xarm5_gazebo_sim bash -c "source /ros2_ws/install/setup.bash && ros2 topic list"

# Зайти в інтерактивний термінал всередину контейнера:
docker exec -it xarm5_gazebo_sim bash
```
