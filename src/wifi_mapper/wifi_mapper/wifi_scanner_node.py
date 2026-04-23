#!/usr/bin/env python3
"""
wifi_scanner_node — measures WiFi at each point and logs to JSON.

Triggered by explorer_node via /trigger_scan.
Throughput priority: iperf3 (real TCP) > tx bitrate from iw > RSSI estimate.
RSSI is read from /proc/net/wireless (no external tools), with nmcli/iw as fallback.
"""

import json
import os
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class WifiScannerNode(Node):

    def __init__(self):
        super().__init__('wifi_scanner_node')

        self.declare_parameter('interface',       'wlan0')
        self.declare_parameter('use_iperf3',      False)
        self.declare_parameter('iperf3_server',   '192.168.1.1')
        self.declare_parameter('iperf3_duration', 3)
        self.declare_parameter('data_file',       '/root/ros2_ws/wifi_results/wifi_data.json')

        p = self.get_parameter
        self.iface      = self._detect_interface(p('interface').value)
        self.use_iperf3 = bool(p('use_iperf3').value)
        self.iperf3_srv = str(p('iperf3_server').value)
        self.iperf3_dur = int(p('iperf3_duration').value)
        self.data_file  = str(p('data_file').value)

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.measurements: list[dict] = []

        os.makedirs(os.path.dirname(self.data_file), exist_ok=True)
        # Clear previous session data on startup
        if os.path.exists(self.data_file):
            os.remove(self.data_file)

        self.create_subscription(Bool,   'trigger_scan',    self._trigger_cb, 10)
        self.create_subscription(String, 'robot_pose_json', self._pose_cb,    10)

        self.rssi_pub = self.create_publisher(Float32, 'wifi_rssi',       10)
        self.thr_pub  = self.create_publisher(Float32, 'wifi_throughput', 10)

        self.get_logger().info(
            f'WiFi scanner ready | iface: {self.iface} | '
            f'iperf3: {"yes → " + self.iperf3_srv if self.use_iperf3 else "no"} | '
            f'output: {self.data_file}')

    # ── Interface detection ───────────────────────────────────────────

    def _detect_interface(self, configured: str) -> str:
        """Use configured interface if present, otherwise pick the first available."""
        try:
            with open('/proc/net/wireless') as f:
                lines = f.readlines()[2:]
            available = [ln.split()[0].rstrip(':') for ln in lines if ln.strip()]
            if configured in available:
                return configured
            if available:
                chosen = available[0]
                self.get_logger().warn(f'Interface "{configured}" not found — using "{chosen}"')
                return chosen
        except Exception:
            pass
        return configured

    # ── Callbacks ────────────────────────────────────────────────────

    def _pose_cb(self, msg: String):
        data = json.loads(msg.data)
        self.cur_x = float(data['x'])
        self.cur_y = float(data['y'])

    def _trigger_cb(self, msg: Bool):
        if msg.data:
            self._measure()

    # ── WiFi reading (three methods, in order of preference) ─────────

    def _read_proc_wireless(self) -> tuple[float, float]:
        """/proc/net/wireless — always available, no external tools."""
        try:
            with open('/proc/net/wireless') as f:
                lines = f.readlines()
            for line in lines[2:]:
                parts = line.split()
                if parts and parts[0].rstrip(':') == self.iface:
                    rssi = float(parts[3].rstrip('.'))
                    link = float(parts[2].rstrip('.'))
                    if rssi > 0:
                        rssi -= 256   # kernel stores as uint8, convert to signed dBm
                    return rssi, link
        except Exception as e:
            self.get_logger().debug(f'/proc/net/wireless: {e}')
        return -100.0, 0.0

    def _read_nmcli(self) -> tuple[float, float]:
        """nmcli fallback — converts signal % to approximate dBm."""
        try:
            r = subprocess.run(
                ['nmcli', '-t', '-f', 'active,signal', 'dev', 'wifi'],
                capture_output=True, text=True, timeout=5)
            for line in r.stdout.splitlines():
                if line.startswith('yes:'):
                    pct  = int(line.split(':')[1])
                    rssi = (pct / 2.0) - 100.0   # rough conversion: 100% → -50 dBm
                    return rssi, float(pct)
        except Exception as e:
            self.get_logger().debug(f'nmcli: {e}')
        return -100.0, 0.0

    def _read_iw(self) -> tuple[float, float]:
        """iw fallback — also reads tx bitrate when available."""
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
        return -100.0, 0.0

    def _read_wifi(self) -> tuple[float, float]:
        """Try /proc/net/wireless → nmcli → iw. Returns (rssi_dBm, bitrate_Mbps)."""
        for reader in (self._read_proc_wireless, self._read_nmcli, self._read_iw):
            rssi, link = reader()
            if rssi > -100.0:
                return rssi, link
        self.get_logger().warn(
            f'Cannot read WiFi on {self.iface} — check: ip link show | grep wlan')
        return -100.0, 0.0

    def _iperf3_throughput(self) -> float:
        """Real TCP throughput via iperf3 (requires iperf3 -s on the server)."""
        try:
            r = subprocess.run(
                ['iperf3', '-c', self.iperf3_srv, '-t', str(self.iperf3_dur), '-J'],
                capture_output=True, text=True, timeout=self.iperf3_dur + 15)
            bps = json.loads(r.stdout)['end']['sum_received']['bits_per_second']
            return round(bps / 1e6, 2)
        except Exception as e:
            self.get_logger().warn(f'iperf3 error: {e}')
            return 0.0

    @staticmethod
    def _rssi_to_mbps(rssi: float) -> float:
        """
        Estimate throughput from RSSI using 802.11n MCS table (2.4 GHz).
        Used only when iperf3 is off and tx bitrate is unavailable.
        """
        thresholds = [(-50, 150.0), (-55, 120.0), (-60, 90.0), (-65, 54.0),
                      (-70, 36.0),  (-75, 18.0),  (-80, 11.0), (-85, 5.5),
                      (-90, 1.0)]
        for threshold, mbps in thresholds:
            if rssi >= threshold:
                return mbps
        return 0.5   # non-zero floor so heatmap interpolation doesn't break

    # ── Measurement ──────────────────────────────────────────────────

    def _measure(self):
        self.get_logger().info(f'Measuring at ({self.cur_x:.2f}, {self.cur_y:.2f})...')

        rssi, bitrate = self._read_wifi()

        if self.use_iperf3:
            throughput = self._iperf3_throughput()
        elif bitrate > 0:
            throughput = bitrate          # tx bitrate from iw is reliable when available
        else:
            throughput = self._rssi_to_mbps(rssi)

        self.get_logger().info(f'  RSSI: {rssi:.1f} dBm | throughput: {throughput:.1f} Mbps')

        r_msg = Float32(); r_msg.data = float(rssi)
        t_msg = Float32(); t_msg.data = float(throughput)
        self.rssi_pub.publish(r_msg)
        self.thr_pub.publish(t_msg)

        self.measurements.append({
            'x':          round(self.cur_x, 3),
            'y':          round(self.cur_y, 3),
            'rssi':       round(rssi, 1),
            'throughput': round(throughput, 2),
            'timestamp':  round(time.time(), 2),
        })

        try:
            with open(self.data_file, 'w') as f:
                json.dump(self.measurements, f, indent=2)
        except OSError as e:
            self.get_logger().error(f'Write error: {e}')


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
