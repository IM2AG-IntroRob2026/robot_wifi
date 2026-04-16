"""
Launch file principal — WiFi Mapper avec iRobot Create 3.

IMPORTANT : le Create 3 publie ses topics sous un namespace (ex: /robot4/).
            Passez robot_namespace:=robot4 (valeur par défaut).

Usage :
  ros2 launch wifi_mapper wifi_mapper.launch.py
  ros2 launch wifi_mapper wifi_mapper.launch.py robot_namespace:=robot4 room_width:=4.0
  ros2 launch wifi_mapper wifi_mapper.launch.py use_iperf3:=true iperf3_server:=192.168.1.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Arguments de lancement ────────────────────────────────────────
    args = [
        # Namespace du robot Create 3 (voir: ros2 topic list | head -5)
        DeclareLaunchArgument('robot_namespace',    default_value='robot4',
                              description='Namespace du robot (ex: robot4, robot1)'),

        DeclareLaunchArgument('room_width',         default_value='2.0',
                              description='Largeur de la salle (m)'),
        DeclareLaunchArgument('room_height',        default_value='2.0',
                              description='Hauteur/profondeur de la salle (m)'),
        DeclareLaunchArgument('row_spacing',        default_value='0.30',
                              description='Espacement entre les rangées (m)'),
        DeclareLaunchArgument('room_margin',        default_value='0.15',
                              description='Marge par rapport aux bords (m)'),
        DeclareLaunchArgument('linear_speed',       default_value='0.15',
                              description='Vitesse de translation (m/s)'),
        DeclareLaunchArgument('angular_speed',      default_value='0.5',
                              description='Vitesse de rotation (rad/s)'),
        DeclareLaunchArgument('waypoint_tolerance', default_value='0.10',
                              description="Tolérance d'arrivée au waypoint (m)"),
        DeclareLaunchArgument('scan_duration',      default_value='2.5',
                              description='Durée de mesure à chaque point (s)'),

        DeclareLaunchArgument('interface',          default_value='wlan0',
                              description='Interface WiFi (ex: wlan0, wlp2s0)'),
        DeclareLaunchArgument('use_iperf3',         default_value='false',
                              description='Utiliser iperf3 pour mesurer le débit réel'),
        DeclareLaunchArgument('iperf3_server',      default_value='192.168.1.1',
                              description='IP du serveur iperf3'),
        DeclareLaunchArgument('iperf3_duration',    default_value='3',
                              description='Durée du test iperf3 (s)'),

        DeclareLaunchArgument('data_file',          default_value='/root/ros2_ws/wifi_results/wifi_data.json',
                              description='Fichier de données WiFi'),
        DeclareLaunchArgument('output_dir',         default_value='/root/ros2_ws/wifi_results',
                              description='Répertoire de sortie des cartes (monté sur ~/ros2_ws/wifi_results)'),
        DeclareLaunchArgument('resolution',         default_value='120',
                              description="Résolution de la grille d'interpolation"),
    ]

    ns = LaunchConfiguration('robot_namespace')

    # ── Noeud 1 : Explorateur ─────────────────────────────────────────
    # namespace=ns fait que /cmd_vel devient /robot4/cmd_vel,
    # /odom devient /robot4/odom, etc.
    explorer = Node(
        package    = 'wifi_mapper',
        executable = 'explorer_node',
        name       = 'explorer_node',
        namespace  = ns,
        output     = 'screen',
        parameters = [{
            'room_width':         LaunchConfiguration('room_width'),
            'room_height':        LaunchConfiguration('room_height'),
            'row_spacing':        LaunchConfiguration('row_spacing'),
            'room_margin':        LaunchConfiguration('room_margin'),
            'linear_speed':       LaunchConfiguration('linear_speed'),
            'angular_speed':      LaunchConfiguration('angular_speed'),
            'waypoint_tolerance': LaunchConfiguration('waypoint_tolerance'),
            'scan_duration':      LaunchConfiguration('scan_duration'),
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

    # ── Noeud 3 : Générateur de carte thermique ───────────────────────
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
