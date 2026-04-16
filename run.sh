#!/bin/bash
# ============================================================
#  Lancement wifi_mapper — à exécuter dans le container Docker
#
#  Paramètres optionnels (exemples) :
#    bash run.sh room_width:=4.0 room_height:=5.0
#    bash run.sh robot_namespace:=robot4 room_width:=3.0
#    bash run.sh use_iperf3:=true iperf3_server:=192.168.1.100
# ============================================================
source /opt/ros/iron/setup.bash
source /root/ros2_ws/install/setup.bash

echo "=== Topics odom disponibles ==="
ros2 topic list 2>/dev/null | grep odom || echo "  (aucun topic odom visible)"
echo ""

ros2 launch wifi_mapper wifi_mapper.launch.py "$@"
