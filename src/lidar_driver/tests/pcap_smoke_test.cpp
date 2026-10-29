#include "lidar_sdk/api/lidar_driver.hpp"
#include "msg/lidar_point_cloud_msg.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using lidar_sdk::lidar::DriverParam;
using lidar_sdk::lidar::Error;
using lidar_sdk::lidar::InputType;
using lidar_sdk::lidar::LidarDriver;
using lidar_sdk::lidar::LidarImuData;
using lidar_sdk::lidar::LidarType;

namespace {

struct TestState {
  std::atomic<unsigned int> frames{0};
  std::atomic<unsigned int> saved_frames{0};
  std::atomic<unsigned int> imus{0};
  std::atomic<unsigned int> invalid_points{0};
  std::atomic<unsigned int> residual_points{0};
  std::atomic<bool> eof{false};
  std::atomic<bool> fatal_error{false};
  unsigned int max_saved_frames{3};
  std::string model_name;
  std::string output_dir;
  FILE* imu_file{nullptr};
  std::mutex output_mutex;
};

bool hasValidPointCloud(const LidarPointCloudMsg& cloud) {
  for (const auto& point : cloud.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z))
      return true;
  }
  return false;
}

bool hasResidualData(const LidarPointCloudMsg::PointT& point) {
  if (std::isfinite(point.x) || std::isfinite(point.y) || std::isfinite(point.z))
    return false;

#if defined(POINT_TYPE_XYZIRT)
  return point.time != 0.0f;
#elif defined(POINT_TYPE_XYZIRTC)
  return point.range != 0 || point.timestamp != 0.0;
#else
  return false;
#endif
}

void saveCloud(const LidarPointCloudMsg& cloud, const TestState* state,
               unsigned int frame_number) {
  if (state->output_dir.empty() || frame_number > state->max_saved_frames)
    return;

  char path[1024];
  std::snprintf(path, sizeof(path), "%s/%s_frame_%03u.txt",
                state->output_dir.c_str(), state->model_name.c_str(), frame_number);
  FILE* file = std::fopen(path, "w");
  if (!file) {
    std::fprintf(stderr, "[ros1-test] cannot save point cloud: %s\n", path);
    return;
  }

#if defined(POINT_TYPE_XYZIRT)
  for (const auto& point : cloud.points) {
    std::fprintf(file, "%.9f %.9f %.9f %.3f %u %.9f\n",
                 point.x, point.y, point.z, point.intensity,
                 (unsigned int)point.ring, point.time);
  }
#elif defined(POINT_TYPE_XYZIRTC)
  for (const auto& point : cloud.points) {
    std::fprintf(file, "%.9f %.9f %.9f %u %u %u %.9f %.9f\n",
                 point.x, point.y, point.z, (unsigned int)point.intensity,
                 (unsigned int)point.ring, (unsigned int)point.range,
                 point.timestamp, point.rgb);
  }
#else
  for (const auto& point : cloud.points) {
    std::fprintf(file, "%.9f %.9f %.9f %u\n",
                 point.x, point.y, point.z, (unsigned int)point.intensity);
  }
#endif
  std::fclose(file);
}

void onCloud(std::shared_ptr<LidarPointCloudMsg> cloud, TestState* state) {
  if (!cloud || cloud->points.empty()) {
    state->fatal_error.store(true);
    return;
  }

  unsigned int invalid = 0;
  unsigned int residual = 0;
  for (const auto& point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      ++invalid;
      if (hasResidualData(point))
        ++residual;
    }
  }

  state->invalid_points.fetch_add(invalid);
  state->residual_points.fetch_add(residual);
  state->frames.fetch_add(1);
  if (hasValidPointCloud(*cloud) && state->max_saved_frames > 0) {
    unsigned int saved_number = state->saved_frames.fetch_add(1) + 1;
    if (saved_number <= state->max_saved_frames)
      saveCloud(*cloud, state, saved_number);
  }
}

void onImu(const LidarImuData& imu, TestState* state) {
  state->imus.fetch_add(1);
  if (state->imu_file) {
    std::lock_guard<std::mutex> lock(state->output_mutex);
    std::fprintf(state->imu_file, "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                 imu.timestamp, imu.accel_x, imu.accel_y, imu.accel_z,
                 imu.gyro_x, imu.gyro_y, imu.gyro_z, imu.temperature);
    std::fflush(state->imu_file);
  }
}

void onError(const Error& error, TestState* state) {
  std::fprintf(stderr, "[ros1-test] %s\n", error.toString().c_str());
  if (error.error_code == lidar_sdk::lidar::ERRCODE_PCAPEXIT)
    state->eof.store(true);
  if (error.error_code_type == lidar_sdk::lidar::ErrCodeType::ERROR_CODE)
    state->fatal_error.store(true);
}

bool parseModel(const char* value, LidarType* model) {
  if (std::strcmp(value, "V192") == 0 || std::strcmp(value, "v192") == 0) {
    *model = LidarType::V192;
    return true;
  }
  if (std::strcmp(value, "P60") == 0 || std::strcmp(value, "p60") == 0) {
    *model = LidarType::P60;
    return true;
  }
  if (std::strcmp(value, "P60_IMU") == 0 || std::strcmp(value, "p60_imu") == 0) {
    *model = LidarType::P60_IMU;
    return true;
  }
  return false;
}

const char* modelName(LidarType model) {
  return model == LidarType::V192 ? "V192" :
         model == LidarType::P60_IMU ? "P60_IMU" : "P60";
}

void closeOutput(TestState* state) {
  std::lock_guard<std::mutex> lock(state->output_mutex);
  if (state->imu_file) {
    std::fclose(state->imu_file);
    state->imu_file = nullptr;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || argc > 4) {
    std::fprintf(stderr, "usage: %s <V192|P60|P60_IMU> <pcap> [frames]\n", argv[0]);
    return 2;
  }

  LidarType model;
  if (!parseModel(argv[1], &model)) {
    std::fprintf(stderr, "unsupported model: %s\n", argv[1]);
    return 2;
  }

  const unsigned int target_frames = argc == 4
      ? static_cast<unsigned int>(std::strtoul(argv[3], nullptr, 10)) : 5U;
  if (target_frames == 0U) {
    std::fprintf(stderr, "frames must be greater than zero\n");
    return 2;
  }

  TestState state;
  state.model_name = argv[1];
  if (const char* output_dir = std::getenv("ROS1_TEST_OUTPUT_DIR"))
    state.output_dir = output_dir;
  if (const char* save_frames = std::getenv("ROS1_TEST_SAVE_FRAMES"))
    state.max_saved_frames = static_cast<unsigned int>(std::strtoul(save_frames, nullptr, 10));

  if (!state.output_dir.empty() && model == LidarType::P60_IMU) {
    char imu_path[1024];
    std::snprintf(imu_path, sizeof(imu_path), "%s/%s_imu.csv",
                  state.output_dir.c_str(), state.model_name.c_str());
    state.imu_file = std::fopen(imu_path, "w");
    if (state.imu_file) {
      std::fprintf(state.imu_file, "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temperature\n");
    } else {
      std::fprintf(stderr, "[ros1-test] cannot save IMU data: %s\n", imu_path);
    }
  }

  DriverParam param;
  param.lidar_type = model;
  param.input_type = InputType::PCAP_FILE;
  param.input_param.pcap_path = argv[2];
  param.input_param.msop_port = 8899;
  param.input_param.difop_port = 0;
  param.input_param.pcap_repeat = false;
  param.input_param.pcap_rate = 1.0;
  param.decoder_param.wait_for_difop = false;
  param.decoder_param.process_by_thread = true;
  param.decoder_param.min_distance = 0.01f;
  param.decoder_param.max_distance = 300.0f;

  const char* calib_dir = std::getenv("LIDAR_SDK_TEST_CALIB_DIR");
  if (calib_dir && calib_dir[0] != '\0') {
    param.decoder_param.calib_file = std::string(calib_dir) + "/" +
        (model == LidarType::V192 ? "V192" : "P60") +
        "-Correction.json";
  }

  LidarDriver<LidarPointCloudMsg> driver;
  driver.regPointCloudCallback(
      [] { return std::make_shared<LidarPointCloudMsg>(); },
      [&state](std::shared_ptr<LidarPointCloudMsg> cloud) { onCloud(std::move(cloud), &state); });
  driver.regExceptionCallback([&state](const Error& error) { onError(error, &state); });
  driver.regImuCallback([&state](const LidarImuData& imu) { onImu(imu, &state); });

  if (!driver.init(param) || !driver.start()) {
    std::fprintf(stderr, "[ros1-test] init/start failed\n");
    closeOutput(&state);
    return 1;
  }

  const std::time_t deadline = std::time(nullptr) + 120;
  while (std::time(nullptr) < deadline &&
         state.frames.load() < target_frames &&
         !state.eof.load() && !state.fatal_error.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  driver.stop();
  closeOutput(&state);

  const unsigned int frames = state.frames.load();
  const unsigned int imus = state.imus.load();
  const unsigned int invalid = state.invalid_points.load();
  const unsigned int residual = state.residual_points.load();
  std::printf("ROS1_PCAP_RESULT model=%s frames=%u target=%u imu=%u invalid=%u residual=%u eof=%d saved_frames=%u\n",
              modelName(model), frames, target_frames, imus, invalid, residual,
              state.eof.load() ? 1 : 0,
              state.saved_frames.load() < state.max_saved_frames
                  ? state.saved_frames.load() : state.max_saved_frames);

  if (state.fatal_error.load() || frames < target_frames || residual != 0U)
    return 1;
  if (model == LidarType::P60_IMU && imus == 0U) {
    std::fprintf(stderr, "[ros1-test] P60_IMU produced no IMU callbacks\n");
    return 1;
  }
  return 0;
}
