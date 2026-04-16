#!/usr/bin/env python3
"""
Noeud de mesure du débit WiFi.

À chaque déclenchement (/trigger_scan), il :
 1. Lit le RSSI et le bitrate de la carte WiFi (via `iw` ou `iwconfig`)
 2. Optionnellement mesure le débit réel avec iperf3
 3. Sauvegarde la mesure dans un fichier JSON

Topics écoutés:
  /trigger_scan     (std_msgs/Bool)   — déclenche une mesure
  /robot_pose_json  (std_msgs/String) — position courante du robot

Topics publiés:
  /wifi_rssi        (std_msgs/Float32) — RSSI en dBm
  /wifi_throughput  (std_msgs/Float32) — débit estimé en Mbps
"""

import json
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class WifiScannerNode(Node):

    def __init__(self):
        super().__init__('wifi_scanner_node')

        # ── Paramètres ────────────────────────────────────────────────
        self.declare_parameter('interface',       'wlan0')
        self.declare_parameter('use_iperf3',      False)
        self.declare_parameter('iperf3_server',   '192.168.1.1')
        self.declare_parameter('iperf3_duration', 3)
        self.declare_parameter('data_file',       '/tmp/wifi_data.json')

        self.iface      = self._detect_interface(
                              self.get_parameter('interface').value)
        self.use_iperf3 = self.get_parameter('use_iperf3').value
        self.iperf3_srv = self.get_parameter('iperf3_server').value
        self.iperf3_dur = self.get_parameter('iperf3_duration').value
        self.data_file  = self.get_parameter('data_file').value

        # ── État ──────────────────────────────────────────────────────
        self.cur_x = self.cur_y = 0.0
        self.measurements: list[dict] = []

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Bool,   'trigger_scan',    self._trigger_cb, 10)
        self.create_subscription(String, 'robot_pose_json', self._pose_cb,    10)

        # ── Publishers ────────────────────────────────────────────────
        self.rssi_pub = self.create_publisher(Float32, 'wifi_rssi',       10)
        self.thr_pub  = self.create_publisher(Float32, 'wifi_throughput', 10)

        self.get_logger().info(
            f'WiFi scanner prêt — interface: {self.iface} | '
            f'iperf3: {"oui → " + self.iperf3_srv if self.use_iperf3 else "non"}')

    # ── Détection interface ──────────────────────────────────────────

    def _detect_interface(self, configured: str) -> str:
        """
        Si l'interface configurée existe dans /proc/net/wireless, l'utilise.
        Sinon, prend la première interface WiFi disponible.
        """
        try:
            with open('/proc/net/wireless') as f:
                lines = f.readlines()[2:]
            available = [l.split()[0].rstrip(':') for l in lines if l.strip()]
            if configured in available:
                return configured
            if available:
                chosen = available[0]
                self.get_logger().warn(
                    f'Interface "{configured}" absente. '
                    f'Utilisation de "{chosen}" (détectée automatiquement)')
                return chosen
        except Exception:
            pass
        return configured

    # ── Callbacks ────────────────────────────────────────────────────

    def _pose_cb(self, msg: String):
        data = json.loads(msg.data)
        self.cur_x = data['x']
        self.cur_y = data['y']

    def _trigger_cb(self, msg: Bool):
        if msg.data:
            self._measure()

    # ── Mesure WiFi ──────────────────────────────────────────────────

    def _read_proc_wireless(self) -> tuple[float, float]:
        """
        Lit RSSI et qualité depuis /proc/net/wireless.
        Interface kernel — aucun outil externe requis.
        Format : iface: status link. signal. noise. ...
        Returns (rssi_dbm, link_quality)
        """
        try:
            with open('/proc/net/wireless') as f:
                lines = f.readlines()
            for line in lines[2:]:   # sauter les 2 lignes d'en-tête
                parts = line.split()
                if not parts:
                    continue
                if parts[0].rstrip(':') == self.iface:
                    # parts[2] = link quality,  parts[3] = signal level
                    rssi = float(parts[3].rstrip('.'))
                    link = float(parts[2].rstrip('.'))
                    if rssi > 0:
                        rssi -= 256   # valeur brute uint8 → dBm signé
                    return rssi, link
        except Exception as e:
            self.get_logger().debug(f'/proc/net/wireless: {e}')
        return -100.0, 0.0

    def _read_nmcli(self) -> tuple[float, float]:
        """Fallback via nmcli (si disponible)."""
        try:
            r = subprocess.run(
                ['nmcli', '-t', '-f', 'active,signal', 'dev', 'wifi'],
                capture_output=True, text=True, timeout=5)
            for line in r.stdout.splitlines():
                if line.startswith('yes:'):
                    pct = int(line.split(':')[1])
                    rssi = (pct / 2.0) - 100.0   # % → dBm approximatif
                    return rssi, float(pct)
        except Exception as e:
            self.get_logger().debug(f'nmcli: {e}')
        return -100.0, 0.0

    def _read_wifi(self) -> tuple[float, float]:
        """Essaie /proc/net/wireless, puis nmcli, puis iw."""
        rssi, link = self._read_proc_wireless()
        if rssi > -100.0:
            return rssi, link
        rssi, link = self._read_nmcli()
        if rssi > -100.0:
            return rssi, link
        # Dernier recours : iw (si installé)
        try:
            r = subprocess.run(['iw', 'dev', self.iface, 'link'],
                               capture_output=True, text=True, timeout=5)
            sig = re.search(r'signal:\s*(-?\d+)', r.stdout)
            brt = re.search(r'tx bitrate:\s*([\d.]+)', r.stdout)
            rssi = float(sig.group(1)) if sig else -100.0
            link = float(brt.group(1)) if brt else 0.0
            return rssi, link
        except Exception:
            pass
        self.get_logger().warn(
            f'Impossible de lire le WiFi sur {self.iface} — '
            f'vérifiez que le nom est correct (/proc/net/wireless)')
        return -100.0, 0.0

    def _iperf3_throughput(self) -> float:
        """Mesure le débit réel TCP avec iperf3 (en Mbps)."""
        try:
            r = subprocess.run(
                ['iperf3', '-c', self.iperf3_srv,
                 '-t', str(self.iperf3_dur), '-J'],
                capture_output=True, text=True,
                timeout=self.iperf3_dur + 10)
            data = json.loads(r.stdout)
            bps = data['end']['sum_received']['bits_per_second']
            return round(bps / 1e6, 2)
        except Exception as e:
            self.get_logger().warn(f'iperf3 erreur: {e}')
            return 0.0

    @staticmethod
    def _rssi_to_mbps(rssi: float) -> float:
        """
        Estimation du débit 802.11n à partir du RSSI.
        Courbe simplifiée basée sur les MCS 802.11n 2.4 GHz.
        """
        if   rssi >= -50: return 150.0
        elif rssi >= -55: return 120.0
        elif rssi >= -60: return  90.0
        elif rssi >= -65: return  54.0
        elif rssi >= -70: return  36.0
        elif rssi >= -75: return  18.0
        elif rssi >= -80: return  11.0
        elif rssi >= -85: return   5.5
        elif rssi >= -90: return   1.0
        else:             return   0.0

    # ── Mesure principale ────────────────────────────────────────────

    def _measure(self):
        self.get_logger().info(
            f'Mesure en ({self.cur_x:.2f}, {self.cur_y:.2f})...')

        rssi, bitrate = self._read_wifi()

        # Débit : iperf3 en priorité, sinon bitrate TX, sinon estimation RSSI
        if self.use_iperf3:
            throughput = self._iperf3_throughput()
        elif bitrate > 0:
            throughput = bitrate
        else:
            throughput = self._rssi_to_mbps(rssi)

        self.get_logger().info(
            f'  RSSI: {rssi:.1f} dBm | Débit: {throughput:.1f} Mbps')

        # Publier
        r_msg = Float32(); r_msg.data = float(rssi)
        t_msg = Float32(); t_msg.data = float(throughput)
        self.rssi_pub.publish(r_msg)
        self.thr_pub.publish(t_msg)

        # Enregistrer
        entry = {
            'x':          round(self.cur_x, 3),
            'y':          round(self.cur_y, 3),
            'rssi':       rssi,
            'throughput': throughput,
            'timestamp':  round(time.time(), 2),
        }
        self.measurements.append(entry)

        try:
            with open(self.data_file, 'w') as f:
                json.dump(self.measurements, f, indent=2)
        except OSError as e:
            self.get_logger().error(f'Écriture données: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WifiScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
