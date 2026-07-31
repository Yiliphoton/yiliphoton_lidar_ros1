# Yiliphoton LiDAR Driver SDK

## Overview

**yiliphoton_lidar_sdk** is a pure C++ driver library developed by Yiliphoton for V192 / P60 LiDAR sensors. It has no ROS dependency and can be directly integrated into any Linux/Windows project.

```
  +---------------------------+
  |   yiliphoton_lidar_sdk    |
  |                           |
  |  +---------------------+  |
  |  |   Driver Core       |  |
  |  |  (decoder / input)  |  |
  |  +---------------------+  |
  |  |  Software Filter    |  |  ← prebuilt libaglidar_ag1.so
  |  +---------------------+  |
  +---------------------------+
```

### Supported LiDAR Models

| Model | Type |
|-------|------|
| V192  | 192 lines |
| P60   | 96 channels |

### Supported Point Types

| Type      | Fields |
|-----------|--------|
| XYZI      | x, y, z, intensity |
| XYZIRT    | x, y, z, intensity, ring, timestamp, range |
| XYZIRTC   | x, y, z, intensity, ring, timestamp, range, rgb |

---

## Directory Structure

```
yiliphoton_lidar_sdk/
├── src/ylp_driver/          # Driver headers (header-only + some .cpp)
│   ├── api/                 # Public interface: lidar_driver.hpp
│   ├── common/              # Logging, error codes, buffers, etc.
│   ├── driver/
│   │   ├── decoder/V192/    # V192 decoder
│   │   ├── decoder/P60/     # P60 decoder
│   │   ├── algorithm/       # Software filter interface
│   │   └── input/           # UDP / PCAP input
│   └── msg/                 # Point cloud message definitions
├── lib/                     # Prebuilt software filter libraries
│   ├── libaglidar_ag1.so    # x86-64
│   └── arm64/libag1.so      # ARM64
├── demo/                    # Example programs
├── tool/                    # Utility programs
└── CMakeLists.txt
```

---

## Dependencies

```bash
# Required
sudo apt-get install -y libeigen3-dev libpcap-dev libyaml-cpp-dev

# Optional (needed for compiling simple_viewer)
sudo apt-get install -y libpcl-dev
```

---

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Common Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `COMPILE_DEMOS` | ON | Build example programs |
| `COMPILE_TOOLS` | OFF | Build utility programs (requires PCL) |
| `ENABLE_TRANSFORM` | OFF | Enable coordinate transform (requires Eigen) |
| `DISABLE_PCAP_PARSE` | OFF | Disable PCAP parsing (no libpcap needed) |
| `ENABLE_PCL_POINTCLOUD` | OFF | Use PCL point cloud types |
| `ENABLE_POINT_EXTEND_FIELD` | OFF | Output azimuth/elevation extended fields |

Example: build online LiDAR demo only, without PCAP:

```bash
cmake .. -DDISABLE_PCAP_PARSE=ON -DCOMPILE_DEMOS=ON
```

---

## Quick Start

### Online LiDAR

```bash
./demo_online
```

Before running, verify the parameters in `demo_online.cpp` match your actual LiDAR:

```cpp
param.lidar_type = LidarType::V192;   // or LidarType::P60
param.input_param.msop_port  = 8899;  // MSOP port
param.input_param.difop_port = 9988;  // DIFOP port
```

### PCAP Replay

```bash
./demo_pcap
```

Modify the PCAP file path in `demo_pcap.cpp`:

```cpp
param.input_param.pcap_path = "/path/to/your.pcap";
```

---

## Integration into Your Project

Add the `src/` directory to your include path and link the prebuilt library:

```cmake
include_directories(/path/to/yiliphoton_lidar_sdk/src)

find_library(YLP_AG1_LIB aglidar_ag1
  PATHS /path/to/yiliphoton_lidar_sdk/lib)

target_link_libraries(your_target
  ${YLP_AG1_LIB} pthread pcap)
```

Key interface files:

| File | Description |
|------|-------------|
| `ylp_driver/api/lidar_driver.hpp` | Main driver interface |
| `ylp_driver/driver/driver_param.hpp` | Parameter definitions |
| `ylp_driver/msg/point_cloud_msg.hpp` | Point cloud message definition |
| `ylp_driver/common/error_code.hpp` | Error code definitions |

---

## Calibration Files

V192 / P60 LiDARs require calibration files for optimal accuracy. Calibration files are located at:

```
src/ylp_driver/config/V192-Correction.json
src/ylp_driver/config/P60-Correction.json
```

Specify in driver parameters:

```cpp
param.decoder_param.calib_file = "V192-Correction.json";  // for V192
param.decoder_param.calib_file = "P60-Correction.json";   // for P60
```

---

## Software Filter

The SDK includes built-in software filtering provided by the prebuilt library `libaglidar_ag1.so`. Filter parameters are configured through the `algorithm` field in the calibration file, including:

- Crosstalk filter (crosstalk_filter)
- Point cloud noise filter (point_cloud_filter)
- Trailing filter (trailing_filter)
- Blind area filter (blind_area_filter)
- Expansion filter (expansion_filter)

---

## FAQ

**Q: No point cloud data received?**  
Check that the firewall allows the MSOP port (default 8899) and DIFOP port (default 9988).

**Q: `libaglidar_ag1.so` not found during linking?**  
Ensure the `lib/` directory is in the link path, or set `LD_LIBRARY_PATH`:
```bash
export LD_LIBRARY_PATH=/path/to/yiliphoton_lidar_sdk/lib:$LD_LIBRARY_PATH
```

**Q: How to use on ARM platforms?**  
Use `lib/arm64/libag1.so` and modify `PATHS` in CMakeLists.txt to point to `lib/arm64`.
