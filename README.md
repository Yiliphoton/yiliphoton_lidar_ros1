[中文](README_zh.md) | **English**

# Yiliphoton LiDAR ROS1 Driver

## Overview

**yiliphoton_lidar_ros1** is a ROS1 driver package for Yiliphoton V192 / P60 LiDAR sensors, built on top of `yiliphoton_lidar_sdk`. It provides standard ROS1 topic interfaces and works directly with RViz for point cloud visualization.

```
  +----------------------------------+
  |     yiliphoton_lidar_ros1        |
  |                                  |
  |  +----------------------------+  |
  |  |  yiliphoton_lidar (main)   |  |
  |  |  +-----------------------+ |  |
  |  |  |  yiliphoton_lidar_sdk | |  |
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

### Package List

| Package | Description |
|---------|-------------|
| `yiliphoton_lidar` | Main driver node; publishes point cloud and packet topics |
| `depth_make` | Converts point cloud to depth map and RGB images |
| `pcl_ros` | Saves point cloud frames as PCD files |

### Supported Point Types

Select via CMake option `POINT_TYPE` (default: `XYZIRTC`):

| Type     | Fields |
|----------|--------|
| XYZI     | x, y, z, intensity |
| XYZIRT   | x, y, z, intensity, ring, timestamp, range |
| XYZIRTC  | x, y, z, intensity, ring, timestamp, range, rgb |

---

## Directory Structure

```
yiliphoton_lidar_ros1/
├── yiliphoton_lidar/
│   ├── src/
│   │   ├── driver_sdk/          # SDK submodule
│   │   ├── manager/             # Node manager
│   │   ├── source/              # Data source adapter layer
│   │   ├── msg/                 # Message conversion
│   │   └── utility/             # YAML reader, etc.
│   ├── node/                    # Node entry point
│   ├── msg/YlplidarPacket.msg   # Custom packet message
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
mkdir -p ~/ylp_ros1_ws/src
cd ~/ylp_ros1_ws/src

# Place the packages from this repo into src
# Option 1: Copy directly
cp -r /path/to/yiliphoton_lidar_ros1/* .

# Option 2: Symlink
ln -s /path/to/yiliphoton_lidar_ros1/yiliphoton_lidar .
ln -s /path/to/yiliphoton_lidar_ros1/depth_make .
ln -s /path/to/yiliphoton_lidar_ros1/pcl_ros .
```

### Build

```bash
cd ~/ylp_ros1_ws

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

# Enable IMU motion compensation
catkin_make -DENABLE_IMU=ON
```

---

## Configuration

The configuration file is located at `yiliphoton_lidar/config/config.yaml`. After installation, it is at:

```
~/<ws>/devel/share/yiliphoton_lidar/config/config.yaml
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
      lidar_type: V192         # LiDAR model: V192 or P60
      msop_port: 8899          # MSOP data port
      difop_port: 9988         # DIFOP config port
      min_distance: 0.01       # Minimum valid distance (meters)
      max_distance: 500        # Maximum valid distance (meters)
      use_lidar_clock: false   # false: use host timestamp
      pcap_path: /path/to/file.pcap
      process_by_thread: true  # Use separate thread for point cloud processing
    ros:
      ros_frame_id: ylplidar
      ros_recv_packet_topic: /ylplidar_packets
      ros_send_packet_topic: /ylplidar_packets
      ros_send_point_cloud_topic: /ylplidar_points
```

---

## Running

### Connect to Online LiDAR

```bash
source devel/setup.bash
roslaunch yiliphoton_lidar start.launch
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
roslaunch yiliphoton_lidar start.launch
```

### View Point Cloud (RViz)

```bash
rviz
```

In RViz, add a `PointCloud2` display, set the topic to `/ylplidar_points`, and set Fixed Frame to `ylplidar`.

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ylplidar_points` | `sensor_msgs/PointCloud2` | Point cloud data |
| `/ylplidar_packets` | `yiliphoton_lidar/YlplidarPacket` | Raw packets (for recording) |

---

## rosbag Recording and Replay

### Recording

```bash
# Record packets (smaller file size, recommended)
rosbag record /ylplidar_packets -O packets.bag

# Record point cloud (larger file size)
rosbag record /ylplidar_points -O pointcloud.bag
```

### Replay

Modify `config.yaml`: set `msg_source` to `2`, and ensure `ros_recv_packet_topic` matches the recorded topic. Then:

```bash
# Start the driver node first
roslaunch yiliphoton_lidar start.launch &

# Then replay the rosbag
rosbag play packets.bag
```

---

## Depth Map Utility

The `depth_make` node subscribes to `/ylplidar_points` and publishes depth maps and RGB images:

```bash
rosrun depth_make depth_make
```

---

## PCD Save Utility

The `bag_to_pcd` node subscribes to `/ylplidar_points` and saves each point cloud frame as a PCD file:

```bash
rosrun pcl_ros bag_to_pcd
```

---

## Calibration Files

V192/P60 calibration files are located in the SDK source at `src/driver_sdk/src/ylp_driver/config/`. The driver automatically loads the appropriate calibration file based on `lidar_type`; no manual specification is needed in `config.yaml`. Modifications to the JSON files take effect after restarting the node — no recompilation required.

---

## FAQ

**Q: No point cloud received?**  
Check that the firewall allows UDP ports 8899 (MSOP) and 9988 (DIFOP):
```bash
sudo ufw allow 8899/udp
sudo ufw allow 9988/udp
```

**Q: catkin_make reports missing `ylp_driver/...` headers?**  
Ensure the `src/driver_sdk` directory exists and is populated (not an empty symlink). If using a symlink, verify the target path is accessible on the current system.

**Q: `libaglidar_ag1.so` not found at link time?**  
Set the library path at runtime:
```bash
export LD_LIBRARY_PATH=/path/to/yiliphoton_lidar_sdk/lib:$LD_LIBRARY_PATH
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
