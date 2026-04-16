#!/bin/bash
# ============================================================
#  Installation des dépendances — wifi_mapper
#  À exécuter UNE SEULE FOIS à l'intérieur du container Docker
#  (ros-iron-cyclone)
# ============================================================
set -e

echo "=== Dépendances Python ==="
pip3 install matplotlib scipy numpy

echo ""
echo "=== Outils WiFi ==="
apt-get update -qq
apt-get install -y iw wireless-tools iperf3

echo ""
echo "=== Dépendances ROS 2 Iron ==="
apt-get install -y \
    ros-iron-launch \
    ros-iron-launch-ros \
    ros-iron-geometry-msgs \
    ros-iron-nav-msgs \
    ros-iron-std-msgs

echo ""
echo "=== irobot_create_msgs (Create 3) ==="
apt-get install -y ros-iron-irobot-create-msgs 2>/dev/null \
    || echo "  Pas trouvé en binaire — on continue sans (le code reste fonctionnel)"

echo ""
echo "=== Build ==="
source /opt/ros/iron/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select wifi_mapper

echo ""
echo "=== Dépendances installées. ==="
echo "Prochain lancement :"
echo "  source /opt/ros/iron/setup.bash && source /root/ros2_ws/install/setup.bash"
echo "  ros2 launch wifi_mapper wifi_mapper.launch.py"
