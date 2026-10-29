#pragma once

#include "source/source_driver.hpp"


#include "msg/ros_msg/lidar_packet.hpp"
#include <ros/ros.h>

namespace lidar_sdk
{
namespace lidar
{

inline Packet toSdkMsg(const lidar_driver::LidarPacket& ros_msg)
{
  Packet sdk_msg;
  sdk_msg.timestamp = ros_msg.header.stamp.toSec();
  sdk_msg.seq = ros_msg.header.seq;
  sdk_msg.is_difop = ros_msg.is_difop;
  sdk_msg.is_frame_begin = ros_msg.is_frame_begin; 

  for (size_t i = 0; i < ros_msg.data.size(); i++)
  {
    sdk_msg.buf_.emplace_back(ros_msg.data[i]);
  }

  return sdk_msg;
}

class SourcePacketRos : public SourceDriver
{ 
public: 

  virtual void init(const YAML::Node& config);

  SourcePacketRos();

private:

  void putPacket(const lidar_driver::LidarPacket& msg);
  ros::Subscriber pkt_sub_;

  std::unique_ptr<ros::NodeHandle> nh_;
};

SourcePacketRos::SourcePacketRos()
  : SourceDriver(SourceType::MSG_FROM_ROS_PACKET)
{
}

void SourcePacketRos::init(const YAML::Node& config)
{
  SourceDriver::init(config);

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

  std::string ros_recv_topic;
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", 
      ros_recv_topic, "/lidar_packets");

  pkt_sub_ = nh_->subscribe(ros_recv_topic, 100, &SourcePacketRos::putPacket, this);

} 

void SourcePacketRos::putPacket(const lidar_driver::LidarPacket& msg)
{
  driver_ptr_->decodePacket(toSdkMsg(msg));
}

inline lidar_driver::LidarPacket toRosMsg(const Packet& sdk_msg, const std::string& frame_id)
{
  lidar_driver::LidarPacket ros_msg;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(sdk_msg.timestamp);
  ros_msg.header.seq = sdk_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.is_difop = sdk_msg.is_difop;
  ros_msg.is_frame_begin = sdk_msg.is_frame_begin;

  for (size_t i = 0; i < sdk_msg.buf_.size(); i++)
  {
    ros_msg.data.emplace_back(sdk_msg.buf_[i]);
  }

  return ros_msg;
}

class DestinationPacketRos : public DestinationPacket
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPacket(const Packet& msg);
  virtual ~DestinationPacketRos() = default;

private:

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Publisher pkt_pub_;
  std::string frame_id_;
};

inline void DestinationPacketRos::init(const YAML::Node& config)
{
  std::string legacy_frame_id;
  yamlRead<std::string>(config["ros"], "ros_frame_id", legacy_frame_id, "lidar");
  yamlRead<std::string>(config["ros"], "ros_lidar_frame_id", frame_id_, legacy_frame_id);

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", 
      ros_send_topic, "lidar_packets");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pkt_pub_ = nh_->advertise<lidar_driver::LidarPacket>(ros_send_topic, 100);
}

inline void DestinationPacketRos::sendPacket(const Packet& msg)
{
  pkt_pub_.publish(toRosMsg(msg, frame_id_));
}

}  // namespace lidar
}  // namespace lidar_sdk


#ifdef ROS2_FOUND
#include "lidar_driver/LidarPacket.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace lidar_sdk
{
namespace lidar
{

inline Packet toSdkMsg(const lidar_driver::msg::LidarPacket& ros_msg)
{
  Packet sdk_msg;
  sdk_msg.timestamp = ros_msg.header.stamp.sec + double(ros_msg.header.stamp.nanosec) / 1e9;
  //sdk_msg.seq = ros_msg.header.seq;
  sdk_msg.is_difop = ros_msg.is_difop;
  sdk_msg.is_frame_begin = ros_msg.is_frame_begin; 

  for (size_t i = 0; i < ros_msg.data.size(); i++)
  {
    sdk_msg.buf_.emplace_back(ros_msg.data[i]);
  }

  return sdk_msg;
}

class SourcePacketRos : public SourceDriver
{ 
public: 

  virtual void init(const YAML::Node& config);

  SourcePacketRos();

  virtual void AGspinsome() override{rclcpp::spin_some(node_ptr_);}

private:

  void putPacket(const lidar_driver::msg::LidarPacket::SharedPtr msg) const;

  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Subscription<lidar_driver::msg::LidarPacket>::SharedPtr pkt_sub_;
};

SourcePacketRos::SourcePacketRos()
  : SourceDriver(SourceType::MSG_FROM_ROS_PACKET)
{
}

void SourcePacketRos::init(const YAML::Node& config)
{
  SourceDriver::init(config);

  std::string ros_recv_topic;
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", 
      ros_recv_topic, "/lidar_packets");

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "lidar_packets_source_" << node_index++;

  node_ptr_.reset(new rclcpp::Node(node_name.str()));
  pkt_sub_ = node_ptr_->create_subscription<lidar_driver::msg::LidarPacket>(ros_recv_topic, 100, 
      std::bind(&SourcePacketRos::putPacket, this, std::placeholders::_1));
} 

void SourcePacketRos::putPacket(const lidar_driver::msg::LidarPacket::SharedPtr msg) const
{
  driver_ptr_->decodePacket(toSdkMsg(*msg));
}

inline lidar_driver::msg::LidarPacket toRosMsg(const Packet& sdk_msg, const std::string& frame_id)
{
  lidar_driver::msg::LidarPacket ros_msg;
  ros_msg.header.stamp.sec = (uint32_t)floor(sdk_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((sdk_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  //ros_msg.header.seq = sdk_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.is_difop = sdk_msg.is_difop;
  ros_msg.is_frame_begin = sdk_msg.is_frame_begin;

  for (size_t i = 0; i < sdk_msg.buf_.size(); i++)
  {
    ros_msg.data.emplace_back(sdk_msg.buf_[i]);
  }

  return ros_msg;
}

class DestinationPacketRos : public DestinationPacket
{
public:

  virtual void init(const YAML::Node& config);
  virtual void sendPacket(const Packet& msg);
  virtual ~DestinationPacketRos() = default;

private:

  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<lidar_driver::msg::LidarPacket>::SharedPtr pkt_pub_;
  std::string frame_id_;
};

inline void DestinationPacketRos::init(const YAML::Node& config)
{
  std::string legacy_frame_id;
  yamlRead<std::string>(config["ros"], "ros_frame_id", legacy_frame_id, "lidar");
  yamlRead<std::string>(config["ros"], "ros_lidar_frame_id", frame_id_, legacy_frame_id);

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", 
      ros_send_topic, "/lidar_packets");

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "lidar_packets_destination_" << node_index++;

  node_ptr_.reset(new rclcpp::Node(node_name.str()));
  pkt_pub_ = node_ptr_->create_publisher<lidar_driver::msg::LidarPacket>(ros_send_topic, 100);
}

inline void DestinationPacketRos::sendPacket(const Packet& msg)
{
  pkt_pub_->publish(toRosMsg(msg, frame_id_));
}

}  // namespace lidar
}  // namespace lidar_sdk

#endif  // ROS2_FOUND
