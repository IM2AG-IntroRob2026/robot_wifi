#!/usr/bin/env python3
"""
heatmap_node — generates wifi_heatmap.png when exploration finishes.

Triggered by /exploration_done from explorer_node.
Produces a two-panel figure:
  left  — actual measurement points + robot path, colored by throughput
  right — smooth interpolated throughput map (scipy cubic griddata)

Output: /root/ros2_ws/wifi_results/wifi_heatmap.png  (also a summary.json)
"""

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import matplotlib
    matplotlib.use('Agg')   # no display needed — saves to file
    import matplotlib.pyplot as plt
    import numpy as np
    from scipy.interpolate import griddata
    PLOT_OK = True
except ImportError:
    PLOT_OK = False


class HeatmapNode(Node):

    def __init__(self):
        super().__init__('heatmap_node')

        self.declare_parameter('data_file',  '/root/ros2_ws/wifi_results/wifi_data.json')
        self.declare_parameter('output_dir', '/root/ros2_ws/wifi_results')
        self.declare_parameter('resolution', 150)

        self.data_file  = self.get_parameter('data_file').value
        self.output_dir = self.get_parameter('output_dir').value
        self.resolution = self.get_parameter('resolution').value

        os.makedirs(self.output_dir, exist_ok=True)
        self.create_subscription(Bool, 'exploration_done', self._done_cb, 10)

        if not PLOT_OK:
            self.get_logger().error(
                'Missing deps: pip install matplotlib scipy numpy')
        self.get_logger().info(f'Heatmap node ready — output: {self.output_dir}')

    def _done_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Generating WiFi map...')
            self._generate()

    def _load(self):
        if not os.path.exists(self.data_file):
            self.get_logger().error(f'Data file not found: {self.data_file}')
            return None
        with open(self.data_file) as f:
            return json.load(f)

    def _generate(self):
        if not PLOT_OK:
            return

        data = self._load()
        if not data or len(data) < 4:
            self.get_logger().error(
                f'Need at least 4 measurements, got {len(data) if data else 0}')
            return

        # Sort by time to draw the path in the correct order
        data_sorted = sorted(data, key=lambda d: d.get('timestamp', 0))
        path_x = np.array([d['x'] for d in data_sorted])
        path_y = np.array([d['y'] for d in data_sorted])

        xs  = np.array([d['x']          for d in data])
        ys  = np.array([d['y']          for d in data])
        thr = np.array([d['throughput'] for d in data])
        rss = np.array([d['rssi']       for d in data])

        vmin_t, vmax_t = thr.min(), thr.max()

        idx_max = np.argmax(thr)
        idx_min = np.argmin(thr)
        pos_max = (xs[idx_max], ys[idx_max])
        pos_min = (xs[idx_min], ys[idx_min])

        # Build interpolation grid — aspect ratio matches the actual measurement area
        res = self.resolution
        pad = 0.05
        x_range = float(xs.max() - xs.min()) + 2 * pad
        y_range = float(ys.max() - ys.min()) + 2 * pad
        if x_range >= y_range:
            nx, ny = res, max(10, int(res * y_range / x_range))
        else:
            ny, nx = res, max(10, int(res * x_range / y_range))

        xi = np.linspace(xs.min() - pad, xs.max() + pad, nx)
        yi = np.linspace(ys.min() - pad, ys.max() + pad, ny)
        Xi, Yi = np.meshgrid(xi, yi)

        def interp(vals):
            # Cubic is smooth but can produce NaN outside the convex hull;
            # fill those with nearest-neighbor so the full grid is covered.
            Z = griddata((xs, ys), vals, (Xi, Yi), method='cubic')
            mask = np.isnan(Z)
            if mask.any():
                Z[mask] = griddata((xs, ys), vals, (Xi, Yi), method='nearest')[mask]
            return Z

        Z_thr = interp(thr)

        i_max_z = np.unravel_index(np.nanargmax(Z_thr), Z_thr.shape)
        i_min_z = np.unravel_index(np.nanargmin(Z_thr), Z_thr.shape)
        pos_max_z = (float(Xi[i_max_z]), float(Yi[i_max_z]))
        pos_min_z = (float(Xi[i_min_z]), float(Yi[i_min_z]))

        # Figure size scaled to the room aspect ratio
        aspect  = x_range / max(y_range, 1e-3)
        panel_h = 7.0
        panel_w = panel_h * aspect
        fig_w   = max(14.0, panel_w * 2 + 4.0)
        fig     = plt.figure(figsize=(fig_w, panel_h + 2.5), facecolor='#111827')
        gs      = fig.add_gridspec(1, 2, wspace=0.35, left=0.06, right=0.96,
                                   top=0.88, bottom=0.10)

        DARK  = '#111827'
        FLOOR = '#1e293b'
        GRID  = '#334155'
        WHITE = '#f1f5f9'

        # ── Left panel: measured points + robot path ──────────────────
        ax1 = fig.add_subplot(gs[0])
        ax1.set_facecolor(FLOOR)

        margin = 0.12
        room = plt.Rectangle(
            (xs.min() - margin, ys.min() - margin),
            xs.max() - xs.min() + 2*margin,
            ys.max() - ys.min() + 2*margin,
            fill=True, facecolor='#0f172a',
            edgecolor='#475569', linewidth=1.5, linestyle='--', zorder=0)
        ax1.add_patch(room)

        ax1.plot(path_x, path_y, color='#475569', linewidth=1.2, zorder=1)

        # Direction arrows along the path (every ~N/12 points)
        step = max(1, len(path_x) // 12)
        for i in range(0, len(path_x) - 1, step):
            dx, dy = path_x[i+1] - path_x[i], path_y[i+1] - path_y[i]
            ax1.annotate('', xy=(path_x[i]+dx*0.6, path_y[i]+dy*0.6),
                         xytext=(path_x[i], path_y[i]),
                         arrowprops=dict(arrowstyle='->', color='#64748b', lw=1.0))

        sc = ax1.scatter(xs, ys, c=thr, cmap='RdYlGn', s=120,
                         vmin=vmin_t, vmax=vmax_t,
                         edgecolors='#1e293b', linewidths=0.8, zorder=3)

        # Sequence numbers every 3rd point
        for i, d in enumerate(data_sorted):
            if i % 3 == 0:
                ax1.text(d['x'], d['y'], str(i+1),
                         color='white', fontsize=5, ha='center', va='center',
                         fontweight='bold', zorder=4)

        ax1.scatter(*pos_max, s=350, marker='*', c='#4ade80', zorder=6,
                    edgecolors='black', linewidths=1.0)
        ax1.scatter(*pos_min, s=300, marker='v', c='#f87171', zorder=6,
                    edgecolors='black', linewidths=1.0)

        rw = xs.max() - xs.min() + 2*margin
        rh = ys.max() - ys.min() + 2*margin
        ann = dict(fontsize=8, fontweight='bold', zorder=7,
                   bbox=dict(boxstyle='round,pad=0.3', alpha=0.8))
        ax1.annotate(f'MAX\n{thr[idx_max]:.0f} Mbps', xy=pos_max,
                     xytext=(pos_max[0]+rw*0.12, pos_max[1]+rh*0.10),
                     color='#4ade80',
                     arrowprops=dict(arrowstyle='->', color='#4ade80', lw=1.2), **ann)
        ax1.annotate(f'MIN\n{thr[idx_min]:.0f} Mbps', xy=pos_min,
                     xytext=(pos_min[0]-rw*0.15, pos_min[1]-rh*0.12),
                     color='#f87171',
                     arrowprops=dict(arrowstyle='->', color='#f87171', lw=1.2), **ann)

        cb1 = fig.colorbar(sc, ax=ax1, shrink=0.80, pad=0.03)
        cb1.set_label('Throughput (Mbps)', color=WHITE, fontsize=10)
        cb1.ax.yaxis.set_tick_params(color=WHITE)
        plt.setp(cb1.ax.yaxis.get_ticklabels(), color=WHITE)

        ax1.set_xlabel('X (m)', color=WHITE, fontsize=11)
        ax1.set_ylabel('Y (m)', color=WHITE, fontsize=11)
        ax1.set_title('Measured points + path', color=WHITE, fontsize=12,
                      fontweight='bold', pad=8)
        ax1.tick_params(colors=WHITE)
        ax1.grid(True, color=GRID, linestyle='--', linewidth=0.5, alpha=0.6)
        ax1.set_aspect('equal', adjustable='box')
        for sp in ax1.spines.values():
            sp.set_edgecolor(GRID)
        ax1.text(0.02, 0.02, f'{len(data)} points',
                 transform=ax1.transAxes, color='#94a3b8', fontsize=9, va='bottom')

        # ── Right panel: interpolated throughput map ──────────────────
        ax2 = fig.add_subplot(gs[1])
        ax2.set_facecolor('#0f172a')

        thr_spread = float(np.nanmax(Z_thr) - np.nanmin(Z_thr))
        if thr_spread < 0.1:
            # All values identical — use imshow instead of contourf (avoids crash)
            cf = ax2.imshow(Z_thr, origin='lower', aspect='auto',
                            extent=[xi.min(), xi.max(), yi.min(), yi.max()],
                            cmap='RdYlGn', vmin=vmin_t-1, vmax=vmax_t+1, alpha=0.92)
        else:
            cf = ax2.contourf(Xi, Yi, Z_thr, levels=30, cmap='RdYlGn',
                              vmin=vmin_t, vmax=vmax_t, alpha=0.92)
            ax2.contour(Xi, Yi, Z_thr, levels=min(10, max(2, int(thr_spread))),
                        colors='white', linewidths=0.25, alpha=0.3)

        ax2.scatter(xs, ys, c='white', s=35, zorder=5,
                    edgecolors='#334155', linewidths=0.6)

        ax2.scatter(*pos_max_z, s=350, marker='*', c='#4ade80', zorder=8,
                    edgecolors='black', linewidths=1.0)
        ax2.scatter(*pos_min_z, s=300, marker='v', c='#f87171', zorder=8,
                    edgecolors='black', linewidths=1.0)

        x_span = xi.max() - xi.min()
        y_span = yi.max() - yi.min()
        ax2.annotate(f'MAX\n{Z_thr.max():.0f} Mbps', xy=pos_max_z,
                     xytext=(pos_max_z[0]+x_span*0.10, pos_max_z[1]+y_span*0.10),
                     color='#4ade80',
                     arrowprops=dict(arrowstyle='->', color='#4ade80', lw=1.2), **ann)
        ax2.annotate(f'MIN\n{Z_thr.min():.0f} Mbps', xy=pos_min_z,
                     xytext=(pos_min_z[0]-x_span*0.12, pos_min_z[1]-y_span*0.12),
                     color='#f87171',
                     arrowprops=dict(arrowstyle='->', color='#f87171', lw=1.2), **ann)

        cb2 = fig.colorbar(cf, ax=ax2, shrink=0.80, pad=0.03)
        cb2.set_label('Estimated throughput (Mbps)', color=WHITE, fontsize=10)
        cb2.ax.yaxis.set_tick_params(color=WHITE)
        plt.setp(cb2.ax.yaxis.get_ticklabels(), color=WHITE)

        ax2.set_xlabel('X (m)', color=WHITE, fontsize=11)
        ax2.set_ylabel('Y (m)', color=WHITE, fontsize=11)
        ax2.set_title('Interpolated throughput map', color=WHITE, fontsize=12,
                      fontweight='bold', pad=8)
        ax2.tick_params(colors=WHITE)
        ax2.grid(True, color=GRID, linestyle='--', linewidth=0.5, alpha=0.4)
        ax2.set_aspect('equal', adjustable='box')
        for sp in ax2.spines.values():
            sp.set_edgecolor(GRID)

        fig.suptitle(
            f'WiFi map  —  {len(data)} points  '
            f'|  RSSI {rss.min():.0f} → {rss.max():.0f} dBm  '
            f'|  throughput {vmin_t:.0f} → {vmax_t:.0f} Mbps',
            color=WHITE, fontsize=13, fontweight='bold')

        # ── Save ──────────────────────────────────────────────────────
        png_path  = os.path.join(self.output_dir, 'wifi_heatmap.png')
        json_path = os.path.join(self.output_dir, 'summary.json')

        plt.savefig(png_path, dpi=150, bbox_inches='tight', facecolor=DARK)
        plt.close(fig)
        self.get_logger().info(f'Saved: {png_path}')

        summary = {
            'n_measurements': len(data),
            'throughput_max': {'mbps': round(float(thr.max()), 2),
                               'position': {'x': round(float(pos_max[0]), 2),
                                            'y': round(float(pos_max[1]), 2)}},
            'throughput_min': {'mbps': round(float(thr.min()), 2),
                               'position': {'x': round(float(pos_min[0]), 2),
                                            'y': round(float(pos_min[1]), 2)}},
            'rssi_max_dbm': round(float(rss.max()), 1),
            'rssi_min_dbm': round(float(rss.min()), 1),
        }
        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2)

        sep = '═' * 50
        self.get_logger().info(sep)
        self.get_logger().info(f'  MAX: {thr.max():.1f} Mbps at {pos_max}')
        self.get_logger().info(f'  MIN: {thr.min():.1f} Mbps at {pos_min}')
        self.get_logger().info(f'  PNG: {png_path}')
        self.get_logger().info(sep)


def main(args=None):
    rclpy.init(args=args)
    node = HeatmapNode()
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
