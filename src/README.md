# LiDAR ROS1 Driver

## 概述

**lidar_driver_ros1** 是 V192 / P60 / P60_IMU 激光雷达的 ROS1 驱动软件包，基于 `lidar_driver_sdk` 构建，提供标准 ROS1 话题接口，可直接配合 RViz 查看点云。

```
  +----------------------------------+
  |     lidar_driver_ros1        |
  |                                  |
  |  +----------------------------+  |
  |  |   lidar_driver (主包)  |  |
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

### 包列表

| 包名 | 说明 |
|------|------|
| `lidar_driver` | 主驱动节点，发布点云和 Packet 话题 |
| `depth_make` | 将点云转换为深度图和 RGB 图像 |
| `pcl_ros` | 将点云帧保存为 PCD 文件 |

### 支持的点类型

通过 CMake 选项 `POINT_TYPE` 选择（默认 `XYZIRT`）：

| 类型 | 字段 |
|------|------|
| XYZI | x, y, z, intensity |
| XYZIRT | x, y, z, intensity, ring, time |
| XYZIRTC | x, y, z, intensity, ring, range, confidence, timestamp, rgb |

> 字段说明：`time` 是相对当前帧首点的秒偏移（`float`，单位：秒）；`timestamp` 是绝对 Unix 秒（`double`，单位：秒），仅在 `XYZIRTC` 兼容点型中出现。
> 帧头时间使用帧首绝对时间：V192/P60 当前用主机接收时间，P60_IMU 用设备 UTC 时间。


### 支持的雷达型号

| 型号 | 类型 |
|------|------|
| V192 | 192 线 |
| P60  | 96 通道 |
| P60_IMU | 96 通道 + IMU |

---

## 目录结构

```
lidar_driver_ros1/
├── lidar_driver/
│   ├── src/
│   │   ├── driver_sdk/          # SDK 子模块（symlink 或 submodule）
│   │   ├── manager/             # 节点管理器
│   │   ├── source/              # 数据源适配层
│   │   ├── msg/                 # 消息转换
│   │   └── utility/             # YAML 读取等工具
│   ├── node/                    # 节点入口
│   ├── msg/LidarPacket.msg   # 自定义 Packet 消息
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
mkdir -p ~/lidar_ros1_ws/src
cd ~/lidar_ros1_ws/src

# 将本仓库的各个包放入 src 目录
# 方式一：直接复制
cp -r /path/to/lidar_driver_ros1/* .

# 方式二：软链接
ln -s /path/to/lidar_driver_ros1/lidar_driver .
ln -s /path/to/lidar_driver_ros1/depth_make .
ln -s /path/to/lidar_driver_ros1/pcl_ros .
```

### 编译

```bash
cd ~/lidar_ros1_ws

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

```

---

## 配置

配置文件位于 `lidar_driver/config/config.yaml`，安装后位于：

```
~/<ws>/devel/share/lidar_driver/config/config.yaml
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
      lidar_type: V192         # 雷达型号，V192  P60 P60_IMU
      echo_option: 0           # 回波模式：0合并，1第一回波，2第二回波，3双回波
      msop_port: 8899          # MSOP 数据端口
      difop_port: 9988         # DIFOP 配置端口
      min_distance: 0.01       # 最小有效距离（米）
      max_distance: 500        # 最大有效距离（米）
      use_lidar_clock: false   # false: 使用主机时间戳
      pcap_path: /path/to/file.pcap
      process_by_thread: true  # 点云处理是否使用独立线程
    ros:
      ros_frame_id: lidar
      ros_recv_packet_topic: /lidar_packets
      ros_send_packet_topic: /lidar_packets
      ros_send_point_cloud_topic: /lidar_points
      ros_send_imu_topic: /lidar_imu           # IMU 话题（仅 P60_IMU）
```

### 回波选择

`echo_option` 用于选择点云输出模式，配置在 `config.yaml` 的 `lidar.driver` 下，默认值为 `0`。四种模式如下：

| 值 | 模式 | 处理与输出 |
|---:|------|------------|
| `0` | 双回波合并（默认） | 双回波参与处理，合并后输出一个回波（第一回波位置）。 |
| `1` | 第一回波 | 只处理并输出第一回波。 |
| `2` | 第二回波 | 只处理并输出第二回波。 |
| `3` | 双回波 | 双回波参与处理并同时输出，点云按双回波标记。 |

该配置适用于 V192、P60 和 P60_IMU。回波选择由 YAML 配置控制，不再读取标定 JSON 中历史遗留的 `crosstalk_filter.echo_option` 参数；修改后重启节点生效。

---

## 运行

### 连接在线雷达

```bash
source devel/setup.bash
roslaunch lidar_driver start.launch
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
roslaunch lidar_driver start.launch
```

### 查看点云（RViz）

```bash
rviz
```

在 RViz 中添加 `PointCloud2` 显示，话题选择 `/lidar_points`，Fixed Frame 设置为 `lidar`。

---

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/lidar_points` | `sensor_msgs/PointCloud2` | 点云数据 |
| `/lidar_packets` | `lidar_driver/LidarPacket` | 原始 Packet（录制用） |
| `/lidar_imu` | `sensor_msgs/Imu` | IMU 数据（仅 P60_IMU） |

---

## rosbag 录制与回放

### 录制

```bash
# 录制 Packet（体积小，推荐）
rosbag record /lidar_packets -O packets.bag

# 录制点云（体积大）
rosbag record /lidar_points -O pointcloud.bag
```

### 回放

修改 `config.yaml` 将 `msg_source` 设为 `2`，`ros_recv_packet_topic` 与录制时的话题一致，然后：

```bash
# 先启动驱动节点
roslaunch lidar_driver start.launch &

# 再回放 rosbag
rosbag play packets.bag
```

---

## 深度图工具

`depth_make` 节点订阅 `/lidar_points`，发布深度图和 RGB 图像：

```bash
rosrun depth_make depth_make
```

---

## PCD 保存工具

`bag_to_pcd` 节点订阅 `/lidar_points`，将每帧点云保存为 PCD 文件：

```bash
rosrun pcl_ros bag_to_pcd
```

---

## 标定文件

V192/P60/P60_IMU 标定文件位于 SDK 源码的 `src/driver_sdk/src/lidar_sdk/config/` 目录下，驱动会根据 `lidar_type` 自动加载，无需在 `config.yaml` 中手动指定。修改 JSON 文件后重启节点即可生效，无需重新编译。

---

## 常见问题

**Q: 收不到点云？**  
检查防火墙是否放行了 UDP 端口 8899（MSOP）和 9988（DIFOP）：
```bash
sudo ufw allow 8899/udp
sudo ufw allow 9988/udp
```

**Q: catkin_make 报找不到 `lidar_sdk/...` 头文件？**  
确认 `src/driver_sdk` 目录存在且内容完整（不是空的 symlink）。如果使用 symlink，确保 symlink 目标路径在当前系统上可访问。

**Q: 链接时找不到 `liblidar_algo.so`？**  
运行时需要设置库路径：
```bash
export LD_LIBRARY_PATH=/path/to/lidar_driver_sdk/lib:$LD_LIBRARY_PATH
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

## LiDAR-IMU 坐标系与外参

对于 P60_IMU：

- 点云消息 `header.frame_id`：`lidar`
- IMU 消息 `header.frame_id`：`imu_link`
- 两个坐标系均为右手坐标系。
- 默认外参（IMU 原点在 LiDAR 坐标系中的位置）：`[0.0000, 0.0183, -0.0345] m`。
- 默认旋转四元数（`x,y,z,w`）：`[0, 0, 0, 1]`。

驱动只负责发布消息及其 `frame_id`，不发布静态 TF，也不对原始点云和 IMU 数值做外参变换。外参 TF 由上层设备配置或 SLAM 启动文件按具体设备发布。

