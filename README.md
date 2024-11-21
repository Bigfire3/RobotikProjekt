# ROS 2 Publisher/Subscriber

## **Funktionalität**
Dieses Projekt umfasst zwei ROS 2 Pakete:
- `publisher_pkg`: Ein einfacher Publisher, der Nachrichten eines bestimmten Typs (`std_msgs/msg/String`) in einem festgelegten Intervall veröffentlicht.
- `subscriber_pkg`: Ein Subscriber, der diese Nachrichten empfängt und in der Konsole ausgibt.

## **Benötigte Abhängigkeiten**
- ROS 2 (getestet mit Humble)
- Python 3.6 oder neuer
- Folgende ROS 2-Pakete:
  - `rclpy`
  - `std_msgs`

## **Anleitung zum Starten**
1. ROS 2 Arbeitsbereich einrichten:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/Bigfire3/RobotikProjekt
   cd ..
   colcon build
   source install/setup.bash
2. Publisher starten:
    ros2 run publisher_pkg publisher_node
3. Subscriber starten
    ros2 run subscriber_pkg subscriber_node
