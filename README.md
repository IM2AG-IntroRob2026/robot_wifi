# wifi_mapper

Drives an iRobot Create 3 in a square, measures WiFi throughput at each stop, and produces a heatmap PNG.

```
start → · → · → · → · → · → (corner)
                                  ↓
(corner) ← · ← · ← · ← · ← · (corner)
    ↓
  (end)                       (corner)
                                  ↑
        · → · → · → · → · → (corner)
```

**Default:** 5 steps × 0.30 m = **1.50 m per side**, 21 measurement points.

---

## Run

**1. Start the container (on the host):**
```bash
sudo docker run -it --net=host --privileged \
  --volume=${HOME}/ros2_ws:/root/ros2_ws \
  --env="DISPLAY=$DISPLAY" \
  --volume="${XAUTHORITY}:/root/.Xauthority" \
  ros-iron-cyclone bash
```

**2. Inside the container:**
```bash
bash /root/ros2_ws/run.sh
```

That's it. The script builds the package and launches all three nodes.

**Output:** `~/ros2_ws/wifi_results/wifi_heatmap.png` (visible on the host machine)

---

## Adjust parameters

Edit [`src/wifi_mapper/config/params.yaml`](src/wifi_mapper/config/params.yaml) or pass arguments:

```bash
# Smaller square
bash /root/ros2_ws/run.sh steps_per_side:=4 step_size:=0.30

# Different robot namespace (check with: ros2 topic list | grep odom)
bash /root/ros2_ws/run.sh robot_namespace:=robot1

# Real throughput measurement with iperf3 (requires iperf3 -s on a server)
bash /root/ros2_ws/run.sh use_iperf3:=true iperf3_server:=192.168.1.100
```

| Parameter | Default | Effect |
|-----------|---------|--------|
| `steps_per_side` | 5 | steps per side of the square |
| `step_size` | 0.30 m | distance between measurement points |
| `scan_duration` | 3.0 s | pause at each point for WiFi measurement |
| `robot_namespace` | robot4 | ROS namespace of the Create 3 |
| `interface` | wlan0 | WiFi interface name |

---

## How it works

Three ROS 2 nodes run simultaneously:

```
explorer_node  ──trigger_scan──►  wifi_scanner_node  ──(saves)──►  wifi_data.json
     │                                                                     │
     └──exploration_done──►  heatmap_node  ◄────────────────────────────────
                                  │
                                  └──►  wifi_heatmap.png
```

- **explorer_node** — sends `drive_distance` / `rotate_angle` actions to the Create 3, records position by dead-reckoning, triggers a WiFi scan at each stop.
- **wifi_scanner_node** — reads RSSI from `/proc/net/wireless` (or falls back to `nmcli` / `iw`), estimates throughput from RSSI using 802.11n MCS table, optionally measures real throughput with iperf3.
- **heatmap_node** — when exploration is done, interpolates the measurements with `scipy.interpolate.griddata` and saves a two-panel PNG (measured points + interpolated map).

---

## Files

```
ros2_ws/
├── run.sh                              ← entry point (run this inside the container)
├── wifi_results/                       ← created at runtime
│   ├── wifi_data.json                  ← raw measurements
│   ├── wifi_heatmap.png                ← output image
│   └── summary.json                    ← max/min positions
└── src/wifi_mapper/
    ├── config/params.yaml              ← all tunable parameters
    ├── launch/wifi_mapper.launch.py    ← wires the three nodes together
    └── wifi_mapper/
        ├── explorer_node.py            ← robot movement + scan triggers
        ├── wifi_scanner_node.py        ← WiFi measurement + data logging
        └── heatmap_node.py             ← PNG generation
```

---

## Troubleshooting

**Robot doesn't move** — check the namespace:
```bash
ros2 topic list | grep odom
# if you see /robot1/odom → use robot_namespace:=robot1
```

**WiFi reads -100 dBm** — check the interface name:
```bash
ip link show | grep -E 'wlan|wlp'
# then: bash run.sh interface:=wlp2s0
```

**`irobot_create_msgs` not found:**
```bash
sudo apt install ros-iron-irobot-create-msgs
```
