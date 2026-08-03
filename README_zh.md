**中文** | [English](README.md)

# Yiliphoton LiDAR ROS1 Driver

## 概述

**yiliphoton_lidar_ros1** 是一力光子 V192 / P60 激光雷达的 ROS1 驱动软件包，基于 `yiliphoton_lidar_sdk` 构建，提供标准 ROS1 话题接口，可直接配合 RViz 查看点云。

```
  +----------------------------------+
  |     yiliphoton_lidar_ros1        |
  |                                  |
  |  +----------------------------+  |
  |  |   yiliphoton_lidar (主包)  |  |
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

### 包列表

| 包名 | 说明 |
|------|------|
| `yiliphoton_lidar` | 主驱动节点，发布点云和 Packet 话题 |
| `depth_make` | 将点云转换为深度图和 RGB 图像 |
| `pcl_ros` | 将点云帧保存为 PCD 文件 |

### 支持的点类型

通过 CMake 选项 `POINT_TYPE` 选择（默认 `XYZIRTC`）：

| 类型 | 字段 |
|------|------|
| XYZI | x, y, z, intensity |
| XYZIRT | x, y, z, intensity, ring, timestamp, range |
| XYZIRTC | x, y, z, intensity, ring, timestamp, range, rgb |

### 支持的雷达型号

| 型号 | 类型 |
|------|------|
| V192 | 192 线 |
| P60  | 96 通道 |

---

## 目录结构

```
yiliphoton_lidar_ros1/
├── yiliphoton_lidar/
│   ├── src/
│   │   ├── driver_sdk/          # SDK 子模块（symlink 或 submodule）
│   │   ├── manager/             # 节点管理器
│   │   ├── source/              # 数据源适配层
│   │   ├── msg/                 # 消息转换
│   │   └── utility/             # YAML 读取等工具
│   ├── node/                    # 节点入口
│   ├── msg/YlplidarPacket.msg   # 自定义 Packet 消息
│   ├── config/config.yaml       # 配置文件
│   ├── launch/start.launch      # 启动文件
│   ├── CMakeLists.txt
│   └── package.xml
├── depth_make/                  # 深度图转换工具
└── pcl_ros/                     # PCD 保存工具
```

---

## 安装依赖

### ROS1 版本支持

| Ubuntu | ROS1 版本 |
|--------|-----------|
| 18.04  | Melodic |
| 20.04  | Noetic |

建议安装 `ros-<distro>-desktop-full`，会自动安装 PCL 等依赖。

### 安装其他依赖

```bash
sudo apt-get install -y libeigen3-dev libpcap-dev libyaml-cpp-dev
```

---

## 编译

### 新建 catkin 工作空间

```bash
mkdir -p ~/ylp_ros1_ws/src
cd ~/ylp_ros1_ws/src

# 将本仓库的各个包放入 src 目录
# 方式一：直接复制
cp -r /path/to/yiliphoton_lidar_ros1/* .

# 方式二：软链接
ln -s /path/to/yiliphoton_lidar_ros1/yiliphoton_lidar .
ln -s /path/to/yiliphoton_lidar_ros1/depth_make .
ln -s /path/to/yiliphoton_lidar_ros1/pcl_ros .
```

### 编译

```bash
cd ~/ylp_ros1_ws

# source ROS1 环境
source /opt/ros/noetic/setup.bash

# 编译
catkin_make -DCMAKE_BUILD_TYPE=Release

# source 工作空间
source devel/setup.bash
```

### 常用编译选项

```bash
# 指定点类型
catkin_make -DPOINT_TYPE=XYZIRT

# 启用坐标变换
catkin_make -DENABLE_TRANSFORM=ON

# 启用 IMU 运动补偿
catkin_make -DENABLE_IMU=ON
```

---

## 配置

配置文件位于 `yiliphoton_lidar/config/config.yaml`，安装后位于：

```
~/<ws>/devel/share/yiliphoton_lidar/config/config.yaml
```

### common 参数

```yaml
common:
  msg_source: 1              # 1: 在线雷达  2: rosbag 回放  3: PCAP 文件
  send_packet_ros: false     # 是否发布 Packet 话题（用于录制 rosbag）
  send_point_cloud_ros: true # 是否发布点云话题
```

### lidar 参数

```yaml
lidar:
  - driver:
      lidar_type: V192         # 雷达型号，V192 或 P60
      msop_port: 8899          # MSOP 数据端口
      difop_port: 9988         # DIFOP 配置端口
      min_distance: 0.01       # 最小有效距离（米）
      max_distance: 500        # 最大有效距离（米）
      use_lidar_clock: false   # false: 使用主机时间戳
      pcap_path: /path/to/file.pcap
      process_by_thread: true  # 点云处理是否使用独立线程
    ros:
      ros_frame_id: ylplidar
      ros_recv_packet_topic: /ylplidar_packets
      ros_send_packet_topic: /ylplidar_packets
      ros_send_point_cloud_topic: /ylplidar_points
```

---

## 运行

### 连接在线雷达

```bash
source devel/setup.bash
roslaunch yiliphoton_lidar start.launch
```

### 回放 PCAP 文件

修改 `config.yaml`：

```yaml
common:
  msg_source: 3
lidar:
  - driver:
      pcap_path: /path/to/your.pcap
      pcap_repeat: true
      pcap_rate: 1.0
```

然后启动节点：

```bash
roslaunch yiliphoton_lidar start.launch
```

### 查看点云（RViz）

```bash
rviz
```

在 RViz 中添加 `PointCloud2` 显示，话题选择 `/ylplidar_points`，Fixed Frame 设置为 `ylplidar`。

---

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/ylplidar_points` | `sensor_msgs/PointCloud2` | 点云数据 |
| `/ylplidar_packets` | `yiliphoton_lidar/YlplidarPacket` | 原始 Packet（录制用） |

---

## rosbag 录制与回放

### 录制

```bash
# 录制 Packet（体积小，推荐）
rosbag record /ylplidar_packets -O packets.bag

# 录制点云（体积大）
rosbag record /ylplidar_points -O pointcloud.bag
```

### 回放

修改 `config.yaml` 将 `msg_source` 设为 `2`，`ros_recv_packet_topic` 与录制时的话题一致，然后：

```bash
# 先启动驱动节点
roslaunch yiliphoton_lidar start.launch &

# 再回放 rosbag
rosbag play packets.bag
```

---

## 深度图工具

`depth_make` 节点订阅 `/ylplidar_points`，发布深度图和 RGB 图像：

```bash
rosrun depth_make depth_make
```

---

## PCD 保存工具

`bag_to_pcd` 节点订阅 `/ylplidar_points`，将每帧点云保存为 PCD 文件：

```bash
rosrun pcl_ros bag_to_pcd
```

---

## 标定文件

V192/P60 标定文件位于 SDK 源码的 `src/driver_sdk/src/ylp_driver/config/` 目录下，驱动会根据 `lidar_type` 自动加载，无需在 `config.yaml` 中手动指定。修改 JSON 文件后重启节点即可生效，无需重新编译。

---

## 常见问题

**Q: 收不到点云？**  
检查防火墙是否放行了 UDP 端口 8899（MSOP）和 9988（DIFOP）：
```bash
sudo ufw allow 8899/udp
sudo ufw allow 9988/udp
```

**Q: catkin_make 报找不到 `ylp_driver/...` 头文件？**  
确认 `src/driver_sdk` 目录存在且内容完整（不是空的 symlink）。如果使用 symlink，确保 symlink 目标路径在当前系统上可访问。

**Q: 链接时找不到 `libaglidar_ag1.so`？**  
运行时需要设置库路径：
```bash
export LD_LIBRARY_PATH=/path/to/yiliphoton_lidar_sdk/lib:$LD_LIBRARY_PATH
```
或者将库路径写入 `/etc/ld.so.conf.d/` 并执行 `sudo ldconfig`。

**Q: Ubuntu 20.04 + Noetic 编译报 C++ 标准问题？**  
在 `catkin_make` 时指定 C++14：
```bash
catkin_make -DCMAKE_CXX_STANDARD=14
```

**Q: 找不到 `roslaunch` 命令？**  
确保已 source ROS1 环境：
```bash
source /opt/ros/noetic/setup.bash
```
