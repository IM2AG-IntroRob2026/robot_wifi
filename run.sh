#!/bin/bash
# ============================================================
#  Lancement wifi_mapper — à exécuter DANS le conteneur Docker
#
#  Usage :
#    bash /root/ros2_ws/run.sh
#    bash /root/ros2_ws/run.sh steps_per_side:=4 step_size:=0.40
#    bash /root/ros2_ws/run.sh robot_namespace:=robot4
#    bash /root/ros2_ws/run.sh use_iperf3:=true iperf3_server:=192.168.1.100
#
#  La heatmap sera dans : /root/ros2_ws/wifi_results/wifi_heatmap.png
# ============================================================

set -e

source /opt/ros/iron/setup.bash

cd /root/ros2_ws
mkdir -p wifi_results

echo ""
echo "══════════════════════════════════════════════"
echo "  WiFi Mapper — Build"
echo "══════════════════════════════════════════════"
colcon build --packages-select wifi_mapper --symlink-install 2>&1 | tail -8

source /root/ros2_ws/install/setup.bash

echo ""
echo "══════════════════════════════════════════════"
echo "  Topics odom disponibles"
echo "══════════════════════════════════════════════"
ros2 topic list 2>/dev/null | grep odom || echo "  (aucun topic odom visible)"
echo ""

ros2 launch wifi_mapper wifi_mapper.launch.py "$@"
