# Yiliphoton LiDAR Driver SDK

## 概述

**yiliphoton_lidar_sdk** 是一力光子为 V192 / P60 激光雷达配套的纯 C++ 驱动库，不依赖 ROS，可直接集成到任意 Linux/Windows 工程中。

```
  +---------------------------+
  |   yiliphoton_lidar_sdk    |
  |                           |
  |  +---------------------+  |
  |  |   Driver Core       |  |
  |  |  (decoder / input)  |  |
  |  +---------------------+  |
  |  |  Software Filter    |  |  ← 预编译 libaglidar_ag1.so
  |  +---------------------+  |
  +---------------------------+
```

### 支持的雷达型号

| 型号 | 类型 |
|------|------|
| V192 | 192 线 |
| P60  | 96 通道 |

### 支持的点类型

| 类型 | 字段 |
|------|------|
| XYZI | x, y, z, intensity |
| XYZIRT | x, y, z, intensity, ring, timestamp, range |
| XYZIRTC | x, y, z, intensity, ring, timestamp, range, rgb |

---

## 目录结构

```
yiliphoton_lidar_sdk/
├── src/ylp_driver/          # 驱动头文件（header-only + 少量 .cpp）
│   ├── api/                 # 对外接口 lidar_driver.hpp
│   ├── common/              # 日志、错误码、缓冲区等
│   ├── driver/
│   │   ├── decoder/V192/    # V192 解码器
│   │   ├── decoder/P60/     # P60 解码器
│   │   ├── algorithm/       # 软件滤波接口
│   │   └── input/           # UDP / PCAP 输入
│   └── msg/                 # 点云消息定义
├── lib/                     # 预编译软件滤波库
│   ├── libaglidar_ag1.so    # x86-64
│   └── arm64/libag1.so      # ARM64
├── demo/                    # 示例程序
├── tool/                    # 工具程序
└── CMakeLists.txt
```

---

## 安装依赖

```bash
# 必需
sudo apt-get install -y libeigen3-dev libpcap-dev libyaml-cpp-dev

# 可选（编译 simple_viewer 工具需要）
sudo apt-get install -y libpcl-dev
```

---

## 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 常用编译选项

| 选项 | 默认 | 说明 |
|------|------|------|
| `COMPILE_DEMOS` | ON | 编译示例程序 |
| `COMPILE_TOOLS` | OFF | 编译工具程序（需要 PCL） |
| `ENABLE_TRANSFORM` | OFF | 启用坐标变换（需要 Eigen） |
| `DISABLE_PCAP_PARSE` | OFF | 禁用 PCAP 解析（不需要 libpcap） |
| `ENABLE_PCL_POINTCLOUD` | OFF | 使用 PCL 点云类型 |
| `ENABLE_POINT_EXTEND_FIELD` | OFF | 输出方位角/俯仰角扩展字段 |

示例：只编译在线雷达 demo，不编译 PCAP 相关：

```bash
cmake .. -DDISABLE_PCAP_PARSE=ON -DCOMPILE_DEMOS=ON
```

---

## 快速上手

### 在线雷达

```bash
./demo_online
```

运行前请确认 `demo_online.cpp` 中的参数与实际雷达一致：

```cpp
param.lidar_type = LidarType::V192;   // 或 LidarType::P60
param.input_param.msop_port  = 8899;   // MSOP 端口
param.input_param.difop_port = 9988;   // DIFOP 端口
```

### 回放 PCAP

```bash
./demo_pcap
```

修改 `demo_pcap.cpp` 中的 pcap 文件路径：

```cpp
param.input_param.pcap_path = "/path/to/your.pcap";
```

---

## 集成到自己的工程

将 `src/` 目录加入 include 路径，链接预编译库即可：

```cmake
include_directories(/path/to/yiliphoton_lidar_sdk/src)

find_library(YLP_AG1_LIB aglidar_ag1
  PATHS /path/to/yiliphoton_lidar_sdk/lib)

target_link_libraries(your_target
  ${YLP_AG1_LIB} pthread pcap)
```

主要接口文件：

| 文件 | 说明 |
|------|------|
| `ylp_driver/api/lidar_driver.hpp` | 驱动主接口 |
| `ylp_driver/driver/driver_param.hpp` | 参数定义 |
| `ylp_driver/msg/point_cloud_msg.hpp` | 点云消息定义 |
| `ylp_driver/common/error_code.hpp` | 错误码定义 |

---

## 标定文件

V192 / P60 雷达需要加载标定文件以获得最佳精度。标定文件位于：

```
src/ylp_driver/config/V192-Correction.json
src/ylp_driver/config/P60-Correction.json
```

在驱动参数中指定：

```cpp
param.decoder_param.calib_file = "V192-Correction.json";  // V192 用
param.decoder_param.calib_file = "P60-Correction.json";   // P60 用
```

---

## 软件滤波

SDK 内置软件滤波功能，由预编译库 `libaglidar_ag1.so` 提供。滤波参数通过标定文件中的 `algorithm` 字段配置，包括：

- 串扰滤波（crosstalk_filter）
- 点云噪点滤波（point_cloud_filter）
- 拖尾滤波（trailing_filter）
- 盲区滤波（blind_area_filter）
- 膨胀滤波（expansion_filter）

---

## 常见问题

**Q: 收不到点云数据？**  
检查防火墙是否放行了 MSOP 端口（默认 8899）和 DIFOP 端口（默认 9988）。

**Q: 链接时找不到 `libaglidar_ag1.so`？**  
确保 `lib/` 目录在链接路径中，或设置 `LD_LIBRARY_PATH`：
```bash
export LD_LIBRARY_PATH=/path/to/yiliphoton_lidar_sdk/lib:$LD_LIBRARY_PATH
```

**Q: ARM 平台如何使用？**  
使用 `lib/arm64/libag1.so`，在 CMakeLists.txt 中修改 `PATHS` 指向 `lib/arm64`。
