"""
Launch file — WiFi Mapper avec iRobot Create 3.

Le robot trace un carré et mesure le débit WiFi à chaque point.
Utilise les action servers natifs du Create 3 (drive_distance, rotate_angle).

Dimensions :
  Côté du carré = steps_per_side × step_size
  Défaut        = 5 × 0.35 m = 1.75 m de côté

Usage :
  ros2 launch wifi_mapper wifi_mapper.launch.py
  ros2 launch wifi_mapper wifi_mapper.launch.py robot_namespace:=robot4
  ros2 launch wifi_mapper wifi_mapper.launch.py steps_per_side:=4 step_size:=0.40
  ros2 launch wifi_mapper wifi_mapper.launch.py use_iperf3:=true iperf3_server:=192.168.1.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        # Namespace du robot Create 3
        # Vérifiez avec : ros2 topic list | head -10
        DeclareLaunchArgument(
            'robot_namespace', default_value='robot4',
            description='Namespace du robot Create 3 (ex: robot4, robot1, \"\")'),

        # Géométrie du carré
        DeclareLaunchArgument(
            'steps_per_side', default_value='5',
            description='Nombre de pas par côté'),
        DeclareLaunchArgument(
            'step_size', default_value='0.30',
            description='Distance entre deux points de mesure (m)'),

        # Vitesses
        DeclareLaunchArgument(
            'linear_speed', default_value='0.15',
            description='Vitesse de déplacement (m/s)'),
        DeclareLaunchArgument(
            'angular_speed', default_value='0.5',
            description='Vitesse de rotation (rad/s)'),

        # Scan
        DeclareLaunchArgument(
            'scan_duration', default_value='3.0',
            description='Durée de mesure WiFi à chaque point (s)'),

        # WiFi
        DeclareLaunchArgument(
            'interface', default_value='wlan0',
            description='Interface WiFi (ex: wlan0, wlp2s0)'),
        DeclareLaunchArgument(
            'use_iperf3', default_value='false',
            description='Utiliser iperf3 pour le débit réel'),
        DeclareLaunchArgument(
            'iperf3_server', default_value='192.168.1.1',
            description='IP du serveur iperf3'),
        DeclareLaunchArgument(
            'iperf3_duration', default_value='3',
            description='Durée du test iperf3 (s)'),

        # Fichiers
        DeclareLaunchArgument(
            'data_file',
            default_value='/root/ros2_ws/wifi_results/wifi_data.json',
            description='Fichier de données WiFi'),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/root/ros2_ws/wifi_results',
            description='Répertoire de sortie des cartes'),
        DeclareLaunchArgument(
            'resolution', default_value='150',
            description="Résolution de la grille d'interpolation"),
    ]

    ns = LaunchConfiguration('robot_namespace')

    # ── Noeud 1 : Explorateur (trajectoire + pilotage) ────────────────
    explorer = Node(
        package    = 'wifi_mapper',
        executable = 'explorer_node',
        name       = 'explorer_node',
        namespace  = ns,
        output     = 'screen',
        parameters = [{
            'steps_per_side': LaunchConfiguration('steps_per_side'),
            'step_size':      LaunchConfiguration('step_size'),
            'linear_speed':   LaunchConfiguration('linear_speed'),
            'angular_speed':  LaunchConfiguration('angular_speed'),
            'scan_duration':  LaunchConfiguration('scan_duration'),
        }],
    )

    # ── Noeud 2 : Scanner WiFi ────────────────────────────────────────
    scanner = Node(
        package    = 'wifi_mapper',
        executable = 'wifi_scanner_node',
        name       = 'wifi_scanner_node',
        namespace  = ns,
        output     = 'screen',
        parameters = [{
            'interface':       LaunchConfiguration('interface'),
            'use_iperf3':      LaunchConfiguration('use_iperf3'),
            'iperf3_server':   LaunchConfiguration('iperf3_server'),
            'iperf3_duration': LaunchConfiguration('iperf3_duration'),
            'data_file':       LaunchConfiguration('data_file'),
        }],
    )

    # ── Noeud 3 : Générateur de heatmap ──────────────────────────────
    heatmap = Node(
        package    = 'wifi_mapper',
        executable = 'heatmap_node',
        name       = 'heatmap_node',
        namespace  = ns,
        output     = 'screen',
        parameters = [{
            'data_file':  LaunchConfiguration('data_file'),
            'output_dir': LaunchConfiguration('output_dir'),
            'resolution': LaunchConfiguration('resolution'),
        }],
    )

    return LaunchDescription(args + [explorer, scanner, heatmap])
