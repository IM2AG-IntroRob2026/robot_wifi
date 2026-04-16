#!/bin/bash
# ============================================================
#  Build du workspace wifi_mapper — à lancer dans le container
# ============================================================
set -e
source /opt/ros/iron/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select wifi_mapper
echo ""
echo "Build OK. Lance maintenant :"
echo "  bash /root/ros2_ws/run.sh"
