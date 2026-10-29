#pragma once

#include "lidar_sdk/common/error_code.hpp"
#include "lidar_sdk/driver/driver_param.hpp"
#include "lidar_sdk/msg/lidar_imu_data.hpp"
#include "lidar_sdk/driver/decoder/member_checker.hpp"
#include "lidar_sdk/driver/decoder/trigon.hpp"
#include "lidar_sdk/driver/decoder/section.hpp"
#include "lidar_sdk/driver/decoder/basic_attr.hpp"
#include "lidar_sdk/driver/decoder/split_strategy.hpp"
#include "lidar_sdk/driver/algorithm/filter.hpp"
#include "lidar_sdk/driver/decoder/reflectivity_compensation.hpp"
#include "lidar_sdk/common/sync_queue.hpp"
#include "lidar_sdk/common/block_buffer.hpp"
#include "lidar_sdk/common/json.hpp"


#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#include <cmath>
#include <array>
#include <functional>
#include <memory>

namespace lidar_sdk
{
namespace lidar
{

enum EchoMode
{
  ECHO_SINGLE = 0,
  ECHO_DUAL
};

struct DecoderConstParam
{
  uint16_t MSOP_LEN;
  uint16_t MSOP2_LEN;
  uint16_t DIFOP_LEN;

  uint8_t MSOP_ID_LEN;
  uint8_t DIFOP_ID_LEN;
  uint8_t MSOP_ID[8];
  uint8_t DIFOP_ID[8];

  uint16_t LASER_NUM;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;

  float DISTANCE_MIN;
  float DISTANCE_MAX;
  float DISTANCE_RES;
  float TEMPERATURE_RES;
};

#define INIT_ONLY_ONCE() \
  static bool init_flag = false; \
  if (init_flag) return param; \
  init_flag = true;

template <typename T_PointCloud>
class Decoder
{
public:

#ifdef ENABLE_TRANSFORM
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) = 0;


  virtual bool processPointCloud(std::shared_ptr<T_PointCloud> cloud) = 0;
  virtual ~Decoder() = default;

  void processDifopPkt(const uint8_t* pkt, size_t size);
  bool processMsopPkt(const uint8_t* pkt, size_t size);


  explicit Decoder(const DecoderConstParam& const_param, const DecoderParam& param);

  float getTemperature();
  bool getDeviceInfo(DeviceInfo& info);
  bool getDeviceStatus(DeviceStatus& status);
  double getPacketDuration();
  double prevPktTs();
  void transformPoint(float& x, float& y, float& z);
  void setIntensity0To255(BlockBuffer& frame);

#ifdef POINT_TYPE_XYZIRTC
  float intensityToPackedRGB(uint8_t inten_0_255);
#endif

  void regCallback(
      const std::function<void(const Error&)>& cb_excep,
      const std::function<void(uint16_t, double)>& cb_split_frame,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_finish);

  void regImuCallback(const std::function<void(const LidarImuData&)>& cb_imu)
  {
    cb_imu_ = cb_imu;
  }

  std::shared_ptr<T_PointCloud> point_cloud_;

  void setReflectivityCompensator(std::shared_ptr<ReflectivityCompensator> compensator) {
    reflectivity_compensator_ = std::move(compensator);
  }

#ifndef UNIT_TEST
protected:
#endif

  double cloudTs();

  std::shared_ptr<BlockBuffer> blockGet(size_t w, size_t h, bool dual);
  void blockPut(std::shared_ptr<BlockBuffer> block, bool stuffed);

  DecoderConstParam const_param_;
  DecoderParam param_;
  std::function<void(uint16_t, double)> cb_split_frame_;
  std::function<void(std::shared_ptr<T_PointCloud>)> cb_finish_;
  std::function<void(const Error&)> cb_excep_;
  std::function<void(const LidarImuData&)> cb_imu_;

#ifdef ENABLE_TRANSFORM
  Eigen::Matrix4d trans_;
#endif

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  double packet_duration_;
  DistanceSection distance_section_;

  EchoMode echo_mode_;
  float temperature_;
  DeviceInfo device_info_;
  DeviceStatus device_status_;

  bool angles_ready_;
  double prev_pkt_ts_;
  double prev_point_ts_;
  double first_point_ts_;

  bool IsCalibrated;
  bool CalibEnabled;

  std::shared_ptr<SoftwareFilter> filter_;

  SyncQueue<std::shared_ptr<BlockBuffer>> free_block_queue_;
  SyncQueue<std::shared_ptr<BlockBuffer>> block_queue_;

  std::shared_ptr<ReflectivityCompensator> reflectivity_compensator_;
};

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::regCallback(
    const std::function<void(const Error&)>& cb_excep,
    const std::function<void(uint16_t, double)>& cb_split_frame,
    const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_finish)
{
  cb_excep_ = cb_excep;
  cb_split_frame_ = cb_split_frame;
  cb_finish_ = cb_finish;
}

template <typename T_PointCloud>
inline Decoder<T_PointCloud>::Decoder(const DecoderConstParam& const_param, const DecoderParam& param)
  : const_param_(const_param)
  , param_(param)
  , packet_duration_(0)
  , distance_section_(const_param.DISTANCE_MIN, const_param.DISTANCE_MAX, param.min_distance, param.max_distance)
  , echo_mode_(ECHO_SINGLE)
  , temperature_(0.0)
  , angles_ready_(false)
  , prev_pkt_ts_(0.0)
  , prev_point_ts_(0.0)
  , first_point_ts_(0.0)
  , IsCalibrated(false)
  , CalibEnabled(true)
{
#ifdef ENABLE_TRANSFORM
  Eigen::AngleAxisd current_rotation_x(param_.transform_param.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(param_.transform_param.elevation, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(param_.transform_param.azimuth, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y,
                                           param_.transform_param.z);
  trans_ = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
#endif
}

template <typename T_PointCloud>
inline float Decoder<T_PointCloud>::getTemperature()
{
  return temperature_;
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::getDeviceInfo(DeviceInfo& info)
{
  memcpy(&info, &device_info_, sizeof(DeviceInfo));
  return true;
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::getDeviceStatus(DeviceStatus& status)
{
  memcpy(&status, &device_status_, sizeof(DeviceStatus));
  return true;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::getPacketDuration()
{
  return packet_duration_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::prevPktTs()
{
  return prev_pkt_ts_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::cloudTs()
{
  return first_point_ts_;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::transformPoint(float& x, float& y, float& z)
{
#ifdef ENABLE_TRANSFORM
  Eigen::Vector4d target_ori(x, y, z, 1);
  Eigen::Vector4d target_rotate = trans_ * target_ori;
  x = target_rotate(0);
  y = target_rotate(1);
  z = target_rotate(2);
#endif
}

#ifdef POINT_TYPE_XYZIRTC
/*template <typename T_PointCloud>
inline float Decoder<T_PointCloud>::intensityToPackedRGB(uint8_t inten_0_255)
{
  float x = std::max(0.f, std::min(255.f, float(inten_0_255))) / 255.f;

  float r = 0.f, g = 0.f, b = 0.f;
  if (x < 0.25f) {
    float k = x / 0.25f;
    r = k * 120.f;        g = k * 10.f;         b = 30.f + k * 45.f;
  } else if (x < 0.50f) {
    float k = (x - 0.25f) / 0.25f;
    r = 120.f + k * 85.f; g = 10.f + k * 55.f;  b = 75.f - k * 35.f;
  } else if (x < 0.75f) {
    float k = (x - 0.50f) / 0.25f;
    r = 205.f + k * 45.f; g = 65.f + k * 90.f;  b = 40.f + k * 20.f;
  } else {
    float k = (x - 0.75f) / 0.25f;
    r = 250.f + k * 5.f;  g = 155.f + k * 100.f;b = 60.f + k * 135.f;
  }

  uint32_t r_int = static_cast<uint32_t>(std::max(0.f, std::min(255.f, std::round(r))));
  uint32_t g_int = static_cast<uint32_t>(std::max(0.f, std::min(255.f, std::round(g))));
  uint32_t b_int = static_cast<uint32_t>(std::max(0.f, std::min(255.f, std::round(b))));

  uint32_t v = (255U << 24) | (r_int << 16) | (g_int << 8) | b_int;
  float out;
  std::memcpy(&out, &v, sizeof(float));
  if (std::isnan(out)) { v &= 0x7FFFFFFF; std::memcpy(&out, &v, sizeof(float)); }
  return out;
}*/

template <typename T_PointCloud>
inline float Decoder<T_PointCloud>::intensityToPackedRGB(uint8_t inten_0_255)
{
  // Reflectivity is an 8-bit value. Build the color map once per point-cloud
  // type, then use a direct lookup in the hot point-conversion loop.
  static const std::array<float, 256> rgb_lut = [] {
    std::array<float, 256> table{};
    for (size_t i = 0; i < table.size(); ++i)
    {
      const float t = static_cast<float>(i);
      float r, g, b;
      if (t < 64.f) {
        const float k = t / 64.f;
        r = 0.f;
        g = k * 40.f;
        b = 80.f + k * 120.f;
      } else if (t < 128.f) {
        const float k = (t - 64.f) / 64.f;
        r = 0.f;
        g = std::min(255.f, 40.f + k * 860.f);
        b = std::max(0.f, 200.f - k * 100.f);
      } else if (t < 192.f) {
        const float k = (t - 128.f) / 64.f;
        r = k * 255.f;
        g = 255.f;
        b = 0.f;
      } else {
        const float k = (t - 192.f) / 63.f;
        r = 255.f;
        g = 255.f - k * 255.f;
        b = 0.f;
      }

      const uint32_t v =
          (uint32_t(static_cast<uint8_t>(std::round(r))) << 16) |
          (uint32_t(static_cast<uint8_t>(std::round(g))) << 8) |
          uint32_t(static_cast<uint8_t>(std::round(b)));
      float out;
      std::memcpy(&out, &v, sizeof(float));
      table[i] = out;
    }
    return table;
  }();

  return rgb_lut[inten_0_255];
}
  
  /*
{
    // ---------- Normalize ----------
    float x = std::clamp(float(inten_0_255) / 255.f, 0.f, 1.f);

    // Gamma > 1，使低反射率颜色更深
    // 1.5~2.0 都可以试，推荐 1.7
    x = std::pow(x, 1.7f);

    // ---------- Turbo polynomial ----------
    float r =
        0.13572138f +
        x * (4.61539260f +
        x * (-42.66032258f +
        x * (132.13108234f +
        x * (-152.94239396f +
        x * 59.28637943f))));

    float g =
        0.09140261f +
        x * (2.19418839f +
        x * (4.84296658f +
        x * (-14.18503333f +
        x * (4.27729857f +
        x * 2.82956604f))));

    float b =
        0.10667330f +
        x * (12.64194608f +
        x * (-60.58204836f +
        x * (110.36276771f +
        x * (-89.90310912f +
        x * 27.34824973f))));

    r = std::clamp(r, 0.f, 1.f);
    g = std::clamp(g, 0.f, 1.f);
    b = std::clamp(b, 0.f, 1.f);

    uint32_t rgb =
        (uint32_t(std::round(r * 255.f)) << 16) |
        (uint32_t(std::round(g * 255.f)) << 8) |
         uint32_t(std::round(b * 255.f));

    float out;
    std::memcpy(&out, &rgb, sizeof(float));
    return out;
}
    */
#endif

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::setIntensity0To255(BlockBuffer& frame)
{
  auto applyToPoints = [](std::vector<RawPointT>& points) {
    for (auto& pt : points)
      pt.reflectivity = static_cast<uint8_t>(pt.reflectivity / 126.0f * 255.0f);
  };

  applyToPoints(frame.points_);
  if (frame.is_dual)
    applyToPoints(frame.points2_);
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::processDifopPkt(const uint8_t* pkt, size_t size)
{
  if (size != this->const_param_.DIFOP_LEN)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGDIFOPLEN)), 1);
    return;
  }

  if (memcmp(pkt, this->const_param_.DIFOP_ID, const_param_.DIFOP_ID_LEN) != 0)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGDIFOPID)), 1);
    return;
  }

  decodeDifopPkt(pkt, size);
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::processMsopPkt(const uint8_t* pkt, size_t size)
{
  constexpr static int CLOUD_POINT_MAX = 1000000;

  if (this->point_cloud_ && (this->point_cloud_->points.size() > CLOUD_POINT_MAX))
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_CLOUDOVERFLOW)), 1);
    this->point_cloud_->points.clear();
  }

  if (param_.wait_for_difop && !angles_ready_)
  {
    DELAY_LIMIT_CALL(cb_excep_(Error(ERRCODE_NODIFOPRECV)), 1);
    return false;
  }

  if (size != this->const_param_.MSOP_LEN && size != this->const_param_.MSOP2_LEN)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPLEN)), 1);
    return false;
  }

  if (memcmp(pkt, this->const_param_.MSOP_ID, this->const_param_.MSOP_ID_LEN) != 0)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPID)), 1);
    return false;
  }

#ifdef ENABLE_CRC32_CHECK
  if (!isCrc32Correct(pkt, size))
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGCRC32)), 1);
    return false;
  }
#endif

  return decodeMsopPkt(pkt, size);
}


template <typename T_PointCloud>
inline std::shared_ptr<BlockBuffer> Decoder<T_PointCloud>::blockGet(size_t w, size_t h, bool dual)
{
  const size_t size = w * h;

  std::shared_ptr<BlockBuffer> block = free_block_queue_.pop();
  if (!block || block->size() != size || (dual && block->points2_.empty()))
    return std::make_shared<BlockBuffer>(w, h, dual);

  block->resetForReuse(w, h, dual);
  return block;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::blockPut(std::shared_ptr<BlockBuffer> block, bool stuffed)
{
  constexpr static int RAW_POINT_CLOUD_POOL_MAX = 100;

  if (!stuffed)
  {
    free_block_queue_.push(block);
    return;
  }

  size_t sz = block_queue_.push(block);
  if (sz > RAW_POINT_CLOUD_POOL_MAX)
    block_queue_.clear();
}

}  // namespace lidar
}  // namespace lidar_sdk
