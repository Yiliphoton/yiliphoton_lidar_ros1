#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ROS1_TEST_IMAGE:-yili-lidar:release-builder}"
V192_PCAP="${V192_PCAP:-$ROOT_DIR/../testdata/v192_demo.pcap}"
P60_IMU_PCAP="${P60_IMU_PCAP:-$ROOT_DIR/../testdata/p60_imu_demo.pcap}"
RESULT_DIR="${RESULT_DIR:-$ROOT_DIR/out/test-results}"
SAVE_FRAMES="${SAVE_FRAMES:-3}"
NO_DOCKER=0

if [[ "${1:-}" == "--no-docker" ]]; then
  NO_DOCKER=1
  shift
fi
if [[ $# -ne 0 ]]; then
  echo "usage: $0 [--no-docker]" >&2
  exit 2
fi

[[ -f "$V192_PCAP" ]] || { echo "PCAP not found: $V192_PCAP" >&2; exit 2; }
[[ -f "$P60_IMU_PCAP" ]] || { echo "PCAP not found: $P60_IMU_PCAP" >&2; exit 2; }
mkdir -p "$RESULT_DIR/ros1_output"

ARCH="$(uname -m)"
case "$ARCH" in
  x86_64|amd64) LIB_ARCH_DIR=x86-64 ;;
  aarch64|arm64) LIB_ARCH_DIR=arm64 ;;
  *) echo "Unsupported native architecture: $ARCH" >&2; exit 2 ;;
esac

run_tests() {
  local root="$1"
  local v192="$2"
  local p60="$3"
  local result_dir="$4"
  local build_root="${ROS1_PCAP_BUILD_ROOT:-$(mktemp -d /tmp/lidar_ros1_pcap_build.XXXXXX)}"
  local exe="$build_root/devel/lib/lidar_driver/lidar_driver_pcap_smoke_test"
  local calib="$root/lidar_driver_ros1/src/lidar_driver/src/driver_sdk/src/lidar_sdk/config"

  source /opt/ros/noetic/setup.bash
  cd "$root/lidar_driver_ros1"
  # 使用独立构建目录，避免复用在其他 Docker 挂载路径下生成的 CMakeCache.txt。
  catkin_make \
    --build "$build_root/build" \
    --force-cmake \
    -DCATKIN_DEVEL_PREFIX="$build_root/devel" \
    -DCMAKE_INSTALL_PREFIX="$build_root/install" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPOINT_TYPE=XYZIRT \
    -DBUILD_PCAP_TEST=ON

  mkdir -p "$result_dir/ros1_output"
  export LIDAR_SDK_TEST_CALIB_DIR="$calib"
  export LD_LIBRARY_PATH="$root/lidar_driver_ros1/src/lidar_driver/src/driver_sdk/lib/$LIB_ARCH_DIR:${LD_LIBRARY_PATH:-}"
  export ROS1_TEST_SAVE_FRAMES="$SAVE_FRAMES"

  set +e
  ROS1_TEST_OUTPUT_DIR="$result_dir/ros1_output" \
    "$exe" V192 "$v192" 5 2>&1 | tee "$result_dir/ros1_v192.log"
  local r1=${PIPESTATUS[0]}
  ROS1_TEST_OUTPUT_DIR="$result_dir/ros1_output" \
    "$exe" P60_IMU "$p60" 10 2>&1 | tee "$result_dir/ros1_p60_imu.log"
  local r2=${PIPESTATUS[0]}
  set -e

  if grep -Eiq 'SIGSEGV|segmentation fault|exit code -11' \
      "$result_dir/ros1_v192.log" "$result_dir/ros1_p60_imu.log"; then
    echo "ROS1 PCAP test failed: crash detected" >&2
    return 1
  fi
  if (( r1 != 0 || r2 != 0 )); then
    echo "ROS1 PCAP test failed: V192=$r1 P60_IMU=$r2" >&2
    return 1
  fi
  echo "ROS1 PCAP tests passed"
}

if (( NO_DOCKER )); then
  run_tests "$ROOT_DIR/.." "$V192_PCAP" "$P60_IMU_PCAP" "$RESULT_DIR"
else
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "$ROOT_DIR/..:/workspace/sdk_release" \
    -v "$V192_PCAP:/data/v192_demo.pcap:ro" \
    -v "$P60_IMU_PCAP:/data/p60_imu_demo.pcap:ro" \
    -v "$RESULT_DIR:/results" \
    "$IMAGE" bash -lc '
      mkdir -p /results
      V192_PCAP=/data/v192_demo.pcap \
      P60_IMU_PCAP=/data/p60_imu_demo.pcap \
      RESULT_DIR=/results \
      SAVE_FRAMES='"$SAVE_FRAMES"' \
      /workspace/sdk_release/lidar_driver_ros1/scripts/test_pcap.sh --no-docker
    '
fi
