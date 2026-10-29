#pragma once

#include "source/source.hpp"

#include <lidar_sdk/api/lidar_driver.hpp>
#include <lidar_sdk/common/sync_queue.hpp>

namespace lidar_sdk
{
namespace lidar
{

class SourceDriver : public Source
{
public:

  virtual void init(const YAML::Node& config);
  virtual void start();
  virtual void stop();
  virtual void regPacketCallback(DestinationPacket::Ptr dst);
  virtual void regImuCallback(DestinationImu::Ptr dst);
  virtual ~SourceDriver();

  SourceDriver(SourceType src_type);

protected:

  std::shared_ptr<LidarPointCloudMsg> getPointCloud(void);
  void putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);
  void putPacket(const Packet& msg);
  void putException(const lidar::Error& msg);
  void processPointCloud();

  std::shared_ptr<lidar::LidarDriver<LidarPointCloudMsg>> driver_ptr_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;
  std::thread point_cloud_process_thread_;
  bool to_exit_process_;
};

SourceDriver::SourceDriver(SourceType src_type)
  : Source(src_type), to_exit_process_(false)
{
}

inline void SourceDriver::init(const YAML::Node& config)
{
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  lidar::DriverParam driver_param;

  // input related
  yamlRead<uint16_t>(driver_config, "msop_port", driver_param.input_param.msop_port, 51180);
  yamlRead<uint16_t>(driver_config, "difop_port", driver_param.input_param.difop_port, 7788);
  yamlRead<std::string>(driver_config, "host_address", driver_param.input_param.host_address, "0.0.0.0");
  yamlRead<std::string>(driver_config, "group_address", driver_param.input_param.group_address, "0.0.0.0");
  yamlRead<bool>(driver_config, "use_vlan", driver_param.input_param.use_vlan, false);
  yamlRead<std::string>(driver_config, "pcap_path", driver_param.input_param.pcap_path, "");
  yamlRead<float>(driver_config, "pcap_rate", driver_param.input_param.pcap_rate, 1);
  yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, true);
  yamlRead<uint16_t>(driver_config, "user_layer_bytes", driver_param.input_param.user_layer_bytes, 0);
  yamlRead<uint16_t>(driver_config, "tail_layer_bytes", driver_param.input_param.tail_layer_bytes, 0);

  // decoder related
  std::string lidar_type;
  yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
  driver_param.lidar_type = strToLidarType(lidar_type);

  // Product-level echo selection. JSON files only contain algorithm parameters;
  // echo selection is controlled by config.yaml and defaults to merged output.
  yamlRead<unsigned int>(driver_config, "echo_option", driver_param.decoder_param.echo_option, 0);
  if (driver_param.decoder_param.echo_option > 3)
  {
    LOG_WARNING << "Invalid echo_option=" << driver_param.decoder_param.echo_option
                << ", fallback to 0" << LOG_REND;
    driver_param.decoder_param.echo_option = 0;
  }

  // decoder
  yamlRead<bool>(driver_config, "wait_for_difop", driver_param.decoder_param.wait_for_difop, true);
  yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.decoder_param.use_lidar_clock, false);
  yamlRead<bool>(driver_config, "write_pkt_ts", driver_param.decoder_param.write_pkt_ts, false);
  yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
  yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
  yamlRead<bool>(driver_config, "dense_points", driver_param.decoder_param.dense_points, false);
  yamlRead<bool>(driver_config, "ts_first_point", driver_param.decoder_param.ts_first_point, false);

  yamlRead<bool>(driver_config, "config_from_file", driver_param.decoder_param.config_from_file, false);
  yamlRead<std::string>(driver_config, "angle_path", driver_param.decoder_param.angle_path, "");
  yamlRead<std::string>(driver_config, "calib_file", driver_param.decoder_param.calib_file, "");

  // Resolve calib_file to an absolute path inside the driver SDK source tree.
  // Changes to JSON files take effect on node restart without recompilation.
  {
    auto &cf = driver_param.decoder_param.calib_file;
    if (!cf.empty() && cf.find('/') == std::string::npos)
    {
      cf = std::string(PROJECT_PATH) + "/src/driver_sdk/src/lidar_sdk/config/" + cf;
    }
    if (cf.empty())
    {
      std::string calib_lidar_type = lidar_type;
      if (lidar_type == "P60_IMU")
      {
        calib_lidar_type = "P60";
      }
      cf = std::string(PROJECT_PATH) + "/src/driver_sdk/src/lidar_sdk/config/" + calib_lidar_type + "-Correction.json";
    }
  }

  yamlRead<bool>(driver_config, "enable_fov_aligned", driver_param.decoder_param.enable_fov_aligned, true);
  yamlRead<bool>(driver_config, "process_by_thread", driver_param.decoder_param.process_by_thread, true);
  yamlRead<bool>(driver_config, "enable_s192line_mode", driver_param.decoder_param.enable_s192line_mode, false);

  // transform
  yamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
  yamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
  yamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
  yamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
  yamlRead<float>(driver_config, "elevation", driver_param.decoder_param.transform_param.elevation, 0);
  yamlRead<float>(driver_config, "azimuth", driver_param.decoder_param.transform_param.azimuth, 0);

  switch (src_type_)
  {
    case SourceType::MSG_FROM_LIDAR:
      driver_param.input_type = InputType::ONLINE_LIDAR;
      break;
    case SourceType::MSG_FROM_PCAP:
      driver_param.input_type = InputType::PCAP_FILE;
      break;
    default:
      driver_param.input_type = InputType::RAW_PACKET;
      break;
  }

  driver_param.print();

  driver_ptr_.reset(new lidar::LidarDriver<LidarPointCloudMsg>());
  driver_ptr_->regPointCloudCallback(std::bind(&SourceDriver::getPointCloud, this), 
      std::bind(&SourceDriver::putPointCloud, this, std::placeholders::_1));
  driver_ptr_->regExceptionCallback(
      std::bind(&SourceDriver::putException, this, std::placeholders::_1));
  point_cloud_process_thread_ = std::thread(std::bind(&SourceDriver::processPointCloud, this));

  if (!driver_ptr_->init(driver_param))
  {
    LOG_ERROR << "Driver Initialize Error...." << LOG_REND;
    exit(-1);
  }
}

inline void SourceDriver::start()
{
  driver_ptr_->start();
}

inline SourceDriver::~SourceDriver()
{
  stop();
}

inline void SourceDriver::stop()
{
  if (driver_ptr_)
  {
    driver_ptr_->stop();
  }

  to_exit_process_ = true;
  if (point_cloud_process_thread_.joinable())
  {
    point_cloud_process_thread_.join();
  }
}

inline std::shared_ptr<LidarPointCloudMsg> SourceDriver::getPointCloud(void)
{
  std::shared_ptr<LidarPointCloudMsg> point_cloud = free_point_cloud_queue_.pop();
  if (point_cloud.get() != NULL)
  {
    return point_cloud;
  }

  return std::make_shared<LidarPointCloudMsg>();
}

inline void SourceDriver::regPacketCallback(DestinationPacket::Ptr dst)
{
  Source::regPacketCallback(dst);

  if (pkt_cb_vec_.size() == 1)
  {
    driver_ptr_->regPacketCallback(
        std::bind(&SourceDriver::putPacket, this, std::placeholders::_1));
  }
}

inline void SourceDriver::putPacket(const Packet& msg)
{
  sendPacket(msg);
}

inline void SourceDriver::regImuCallback(DestinationImu::Ptr dst)
{
  Source::regImuCallback(dst);

  if (imu_cb_vec_.size() == 1)
  {
    driver_ptr_->regImuCallback(
        [this](const lidar::LidarImuData& data) { this->sendImu(data); });
  }
}

void SourceDriver::putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg)
{
  point_cloud_queue_.push(msg);
}

void SourceDriver::processPointCloud()
{
#ifndef WIN32
  pthread_setname_np(pthread_self(), "pub_ros_msg");
#endif

  while (!to_exit_process_)
  {
    std::shared_ptr<LidarPointCloudMsg> msg = point_cloud_queue_.popWait(500000);
    if (msg.get() == NULL)
    {
      continue;
    }

    sendPointCloud(msg);

    free_point_cloud_queue_.push(msg);
  }
}

inline void SourceDriver::putException(const lidar::Error& msg)
{
  switch (msg.error_code_type)
  {
    case lidar::ErrCodeType::INFO_CODE:
      LOG_INFO << msg.toString() << LOG_REND;
      break;
    case lidar::ErrCodeType::WARNING_CODE:
      LOG_WARNING << msg.toString() << LOG_REND;
      break;
    case lidar::ErrCodeType::ERROR_CODE:
      LOG_ERROR << msg.toString() << LOG_REND;
      break;
  }
}

}  // namespace lidar
}  // namespace lidar_sdk
