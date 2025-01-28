# ROS 2 Publisher/Subscriber

## **Funktionalität**
Dieses Projekt umfasst zwei ROS 3 Pakete:
- `publisher_pkg`: Ein einfacher Publisher, der Nachrichten eines bestimmten Typs (`std_msgs/msg/String`) in einem festgelegten Intervall veröffentlicht.
- `subscriber_pkg`: Ein Subscriber, der diese Nachrichten empfängt und in der Konsole ausgibt.
- `laserscan_follower`: Eine einfache Node, welche die Daten des Laserscanners auswertet, um dem nächsten Hindernis zu folgen.
- `line_follower`: Eine einfache Node, welche die Daten der Kamera als Graustufenbild auswertet, um einer hellen Linie auf dunklen Grund zu folgen.

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
    ```bash
    ros2 run publisher_pkg publisher_node
3. Subscriber starten
    ```bash
    ros2 run subscriber_pkg subscriber_node
4. Laserscan Follower starten (2. und 3. nicht vorrausgesetzt)
    ```bash
    ros2 run laserscan_follower drive_with_laserscanner
5. Line Follower starten (2., 3. und 4. nicht vorrausgesetzt)
    ```bash
    ros2 run line_follower drive_with_laserscanner
6. Line Follower und Laserscan Follower starten (2., 3., 4. und 5. nicht vorrausgesetzt)
    ```bash
    ros2 launch my_launch_package multi_node_launch.py

