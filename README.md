# ROS 2 Publisher/Subscriber

## **Funktionalität**
Dieses Projekt umfasst zwei ROS 3 Pakete:
- `publisher_pkg`: Ein einfacher Publisher, der Nachrichten eines bestimmten Typs (`std_msgs/msg/String`) in einem festgelegten Intervall veröffentlicht.
- `subscriber_pkg`: Ein Subscriber, der diese Nachrichten empfängt und in der Konsole ausgibt.
- `laserscan_follower`: Eine einfache Node, welche die Daten des Laserscanners auswertet, um dem nächsten Hindernis zu folgen.

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

2.1 Publisher starten:
    ```bash
    ros2 run publisher_pkg publisher_node

2.2 Subscriber starten
    ```bash
    ros2 run subscriber_pkg subscriber_node
    
2.3 Follower starten (2.1 und 2.2 nicht vorrausgesetzt)
    ```bash
    ros2 run laserscan_follower drive_with_laserscanner
