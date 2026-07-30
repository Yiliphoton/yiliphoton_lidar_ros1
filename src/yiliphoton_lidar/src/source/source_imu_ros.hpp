#pragma once

#include "source/source_driver.hpp"

#ifdef ENABLE_MOTION_COMPENSATION

#include "msg/ros_msg/aglidar_packet.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace asensing
{
namespace lidar
{

class SourceImuRos : public SourceDriver
{ 
public: 

  virtual void init(const YAML::Node& config);

  SourceImuRos();

private:

  void saveImuData(const sensor_msgs::ImuConstPtr msg) const;
  ros::Subscriber imu_sub_;

  std::unique_ptr<ros::NodeHandle> nh_;
};

SourceImuRos::SourceImuRos()
  : SourceDriver(SourceType::MSG_FROM_LIDAR)
{
}

void SourceImuRos::init(const YAML::Node& config)
{
  SourceDriver::init(config);

  driver_ptr_->OpenImuMotionCompensation();

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

  std::string ros_recv_topic;
  yamlRead<std::string>(config["ros"], "ros_recv_imu_topic", ros_recv_topic, "imu");

  imu_sub_ = nh_->subscribe(ros_recv_topic, 100, &SourceImuRos::saveImuData, this);
} 

void SourceImuRos::saveImuData(const sensor_msgs::ImuConstPtr msg) const
{
  size_t sz =  driver_ptr_->ImuDataPut(msg);
  if(sz >= 50)
  {
    driver_ptr_->ImuDataPop();
    driver_ptr_->ImuDataPut(msg);
  }
}

}  // namespace lidar
}  // namespace asensing

#endif  // ENABLE_MOTION_COMPENSATION
