# LiDAR ROS1 Driver

## Overview

**lidar_driver_ros1** is a ROS1 driver package for V192 / P60 / P60_IMU LiDAR sensors, built on top of `lidar_driver_sdk`. It provides standard ROS1 topic interfaces and works directly with RViz for point cloud visualization.

```
  +----------------------------------+
  |     lidar_driver_ros1        |
  |                                  |
  |  +----------------------------+  |
  |  |  lidar_driver (main)   |  |
  |  |  +-----------------------+ |  |
  |  |  |  lidar_driver_sdk | |  |
  |  |  +-----------------------+ |  |
  |  +----------------------------+  |
  |  +------------+ +-------------+  |
  |  | depth_make | |   pcl_ros   |  |
  |  +------------+ +-------------+  |
  +----------------------------------+
              ↕ ROS1 Topics
  +----------------------------------+
  |           RViz / rosbag          |
  +----------------------------------+
```

### Supported LiDAR Models

| Model | Type |
|-------|------|
| V192  | 192 lines |
| P60   | 96 channels |
| P60_IMU | 96 channels + IMU |

### Package List

| Package | Description |
|---------|-------------|
| `lidar_driver` | Main driver node; publishes point cloud and packet topics |
| `depth_make` | Converts point cloud to depth map and RGB images |
| `pcl_ros` | Saves point cloud frames as PCD files |

### Supported Point Types

Select via CMake option `POINT_TYPE` (default: `XYZIRT`):

| Type     | Fields |
|----------|--------|
| XYZI     | x, y, z, intensity |
| XYZIRT   | x, y, z, intensity, ring, time |
| XYZIRTC  | x, y, z, intensity, ring, range, confidence, timestamp, rgb |

> Field notes: `time` is the per-point offset in seconds from the current frame start (`float`, seconds); `timestamp` is absolute Unix time in seconds (`double`, seconds) and only appears in the legacy `XYZIRTC` point type.
> The frame header uses the absolute time of the first point in the frame: V192/P60 currently use host receive time, while P60_IMU uses device UTC time.


---

## Directory Structure

```
lidar_driver_ros1/
├── lidar_driver/
│   ├── src/
│   │   ├── driver_sdk/          # SDK submodule
│   │   ├── manager/             # Node manager
│   │   ├── source/              # Data source adapter layer
│   │   ├── msg/                 # Message conversion
│   │   └── utility/             # YAML reader, etc.
│   ├── node/                    # Node entry point
│   ├── msg/LidarPacket.msg   # Custom packet message
│   ├── config/config.yaml       # Configuration file
│   ├── launch/start.launch      # Launch file
│   ├── CMakeLists.txt
│   └── package.xml
├── depth_make/                  # Depth map conversion utility
└── pcl_ros/                     # PCD save utility
```

---

## Dependencies

### Supported ROS1 Versions

| Ubuntu | ROS1 Distro |
|--------|-------------|
| 18.04  | Melodic |
| 20.04  | Noetic |

Installing `ros-<distro>-desktop-full` is recommended; it includes PCL and other dependencies.

### Other Dependencies

```bash
sudo apt-get install -y libeigen3-dev libpcap-dev libyaml-cpp-dev
```

---

## Build

### Create a New catkin Workspace

```bash
mkdir -p ~/lidar_ros1_ws/src
cd ~/lidar_ros1_ws/src

# Place the packages from this repo into src
# Option 1: Copy directly
cp -r /path/to/lidar_driver_ros1/* .

# Option 2: Symlink
ln -s /path/to/lidar_driver_ros1/lidar_driver .
ln -s /path/to/lidar_driver_ros1/depth_make .
ln -s /path/to/lidar_driver_ros1/pcl_ros .
```

### Build

```bash
cd ~/lidar_ros1_ws

# Source ROS1 environment
source /opt/ros/noetic/setup.bash

# Build
catkin_make -DCMAKE_BUILD_TYPE=Release

# Source workspace
source devel/setup.bash
```

### Common Build Options

```bash
# Specify point type
catkin_make -DPOINT_TYPE=XYZIRT

# Enable coordinate transform
catkin_make -DENABLE_TRANSFORM=ON

```

---

## Configuration

The configuration file is located at `lidar_driver/config/config.yaml`. After installation, it is at:

```
~/<ws>/devel/share/lidar_driver/config/config.yaml
```

### Common Parameters

```yaml
common:
  msg_source: 1              # 1: online LiDAR  2: rosbag replay  3: PCAP file
  send_packet_ros: false     # Whether to publish packet topic (for rosbag recording)
  send_point_cloud_ros: true # Whether to publish point cloud topic
```

### LiDAR Parameters

```yaml
lidar:
  - driver:
      lidar_type: V192         # LiDAR model: V192  P60 P60_IMU
      echo_option: 0           # Echo mode: 0=merge, 1=first, 2=second, 3=dual
      msop_port: 8899          # MSOP data port
      difop_port: 9988         # DIFOP config port
      min_distance: 0.01       # Minimum valid distance (meters)
      max_distance: 500        # Maximum valid distance (meters)
      use_lidar_clock: false   # false: use host timestamp
      pcap_path: /path/to/file.pcap
      process_by_thread: true  # Use separate thread for point cloud processing
    ros:
      ros_frame_id: lidar
      ros_recv_packet_topic: /lidar_packets
      ros_send_packet_topic: /lidar_packets
      ros_send_point_cloud_topic: /lidar_points
      ros_send_imu_topic: /lidar_imu           # IMU topic (P60_IMU only)
```

### Echo Selection

`echo_option` selects the point-cloud output mode. Set it under `lidar.driver` in `config.yaml`; the default is `0`.

| Value | Mode | Processing and output |
|------:|------|-----------------------|
| `0` | Dual-echo merge (default) | Process both echoes, merge them, and output one echo at the first-echo position. |
| `1` | First echo | Process and output only the first echo. |
| `2` | Second echo | Process and output only the second echo. |
| `3` | Dual echo | Process both echoes and output both; the point cloud is marked as dual-return. |

This setting applies to V192, P60, and P60_IMU. Echo selection is controlled by YAML and no longer comes from the legacy `crosstalk_filter.echo_option` field in the calibration JSON. Restart the node after changing it.

---

## Running

### Connect to Online LiDAR

```bash
source devel/setup.bash
roslaunch lidar_driver start.launch
```

### PCAP Replay

Modify `config.yaml`:

```yaml
common:
  msg_source: 3
lidar:
  - driver:
      pcap_path: /path/to/your.pcap
      pcap_repeat: true
      pcap_rate: 1.0
```

Then start the node:

```bash
roslaunch lidar_driver start.launch
```

### View Point Cloud (RViz)

```bash
rviz
```

In RViz, add a `PointCloud2` display, set the topic to `/lidar_points`, and set Fixed Frame to `lidar`.

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_points` | `sensor_msgs/PointCloud2` | Point cloud data |
| `/lidar_packets` | `lidar_driver/LidarPacket` | Raw packets (for recording) |
| `/lidar_imu` | `sensor_msgs/Imu` | IMU data (P60_IMU only) |

---

## rosbag Recording and Replay

### Recording

```bash
# Record packets (smaller file size, recommended)
rosbag record /lidar_packets -O packets.bag

# Record point cloud (larger file size)
rosbag record /lidar_points -O pointcloud.bag
```

### Replay

Modify `config.yaml`: set `msg_source` to `2`, and ensure `ros_recv_packet_topic` matches the recorded topic. Then:

```bash
# Start the driver node first
roslaunch lidar_driver start.launch &

# Then replay the rosbag
rosbag play packets.bag
```

---

## Depth Map Utility

The `depth_make` node subscribes to `/lidar_points` and publishes depth maps and RGB images:

```bash
rosrun depth_make depth_make
```

---

## PCD Save Utility

The `bag_to_pcd` node subscribes to `/lidar_points` and saves each point cloud frame as a PCD file:

```bash
rosrun pcl_ros bag_to_pcd
```

---

## Calibration Files

V192/P60/P60_IMU calibration files are located in the SDK source at `src/driver_sdk/src/lidar_sdk/config/`. The driver automatically loads the appropriate calibration file based on `lidar_type`; no manual specification is needed in `config.yaml`. Modifications to the JSON files take effect after restarting the node — no recompilation required.

---

## FAQ

**Q: No point cloud received?**  
Check that the firewall allows UDP ports 8899 (MSOP) and 9988 (DIFOP):
```bash
sudo ufw allow 8899/udp
sudo ufw allow 9988/udp
```

**Q: catkin_make reports missing `lidar_sdk/...` headers?**  
Ensure the `src/driver_sdk` directory exists and is populated (not an empty symlink). If using a symlink, verify the target path is accessible on the current system.

**Q: `liblidar_algo.so` not found at link time?**  
Set the library path at runtime:
```bash
export LD_LIBRARY_PATH=/path/to/lidar_driver_sdk/lib:$LD_LIBRARY_PATH
```
Or add the library path to `/etc/ld.so.conf.d/` and run `sudo ldconfig`.

**Q: C++ standard errors with Ubuntu 20.04 + Noetic?**  
Specify C++14 when running `catkin_make`:
```bash
catkin_make -DCMAKE_CXX_STANDARD=14
```

**Q: `roslaunch` command not found?**  
Ensure you have sourced the ROS1 environment:
```bash
source /opt/ros/noetic/setup.bash
```

## LiDAR-IMU 坐标系与外参

对于 P60_IMU：

- 点云消息 `header.frame_id`：`lidar`
- IMU 消息 `header.frame_id`：`imu_link`
- 两个坐标系均为右手坐标系。
- 默认外参（IMU 原点在 LiDAR 坐标系中的位置）：`[0.0000, 0.0183, -0.0345] m`。
- 默认旋转四元数（`x,y,z,w`）：`[0, 0, 0, 1]`。

驱动只负责发布消息及其 `frame_id`，不发布静态 TF，也不对原始点云和 IMU 数值做外参变换。外参 TF 由上层设备配置或 SLAM 启动文件按具体设备发布。

