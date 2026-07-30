#pragma once

#include "source/source_driver.hpp"


#include "msg/ros_msg/ylplidar_packet.hpp"
#include <ros/ros.h>

namespace asensing
{
namespace lidar
{

inline Packet toAgMsg(const yiliphoton_lidar::YlplidarPacket& ros_msg)
{
  Packet ag_msg;
  ag_msg.timestamp = ros_msg.header.stamp.toSec();
  ag_msg.seq = ros_msg.header.seq;
  ag_msg.is_difop = ros_msg.is_difop;
  ag_msg.is_frame_begin = ros_msg.is_frame_begin; 

  for (size_t i = 0; i < ros_msg.data.size(); i++)
  {
    ag_msg.buf_.emplace_back(ros_msg.data[i]);
  }

  return ag_msg;
}

class SourcePacketRos : public SourceDriver
{ 
public: 

  virtual void init(const YAML::Node& config);

  SourcePacketRos();

private:

  void putPacket(const yiliphoton_lidar::YlplidarPacket& msg);
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
      ros_recv_topic, "/ylplidar_packets");

  pkt_sub_ = nh_->subscribe(ros_recv_topic, 100, &SourcePacketRos::putPacket, this);

} 

void SourcePacketRos::putPacket(const yiliphoton_lidar::YlplidarPacket& msg)
{
  driver_ptr_->decodePacket(toAgMsg(msg));
}

inline yiliphoton_lidar::YlplidarPacket toRosMsg(const Packet& ag_msg, const std::string& frame_id)
{
  yiliphoton_lidar::YlplidarPacket ros_msg;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(ag_msg.timestamp);
  ros_msg.header.seq = ag_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.is_difop = ag_msg.is_difop;
  ros_msg.is_frame_begin = ag_msg.is_frame_begin;

  for (size_t i = 0; i < ag_msg.buf_.size(); i++)
  {
    ros_msg.data.emplace_back(ag_msg.buf_[i]);
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
  yamlRead<std::string>(config["ros"], 
      "ros_frame_id", frame_id_, "ylplidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", 
      ros_send_topic, "aglidar_packets");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pkt_pub_ = nh_->advertise<yiliphoton_lidar::YlplidarPacket>(ros_send_topic, 100);
}

inline void DestinationPacketRos::sendPacket(const Packet& msg)
{
  pkt_pub_.publish(toRosMsg(msg, frame_id_));
}

}  // namespace lidar
}  // namespace asensing


#ifdef ROS2_FOUND
#include "yiliphoton_lidar/YlplidarPacket.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace asensing
{
namespace lidar
{

inline Packet toAgMsg(const yiliphoton_lidar::msg::YlplidarPacket& ros_msg)
{
  Packet ag_msg;
  ag_msg.timestamp = ros_msg.header.stamp.sec + double(ros_msg.header.stamp.nanosec) / 1e9;
  //ag_msg.seq = ros_msg.header.seq;
  ag_msg.is_difop = ros_msg.is_difop;
  ag_msg.is_frame_begin = ros_msg.is_frame_begin; 

  for (size_t i = 0; i < ros_msg.data.size(); i++)
  {
    ag_msg.buf_.emplace_back(ros_msg.data[i]);
  }

  return ag_msg;
}

class SourcePacketRos : public SourceDriver
{ 
public: 

  virtual void init(const YAML::Node& config);

  SourcePacketRos();

  virtual void AGspinsome() override{rclcpp::spin_some(node_ptr_);}

private:

  void putPacket(const yiliphoton_lidar::msg::YlplidarPacket::SharedPtr msg) const;

  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Subscription<yiliphoton_lidar::msg::YlplidarPacket>::SharedPtr pkt_sub_;
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
      ros_recv_topic, "/ylplidar_packets");

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "ylplidar_packets_source_" << node_index++;

  node_ptr_.reset(new rclcpp::Node(node_name.str()));
  pkt_sub_ = node_ptr_->create_subscription<yiliphoton_lidar::msg::YlplidarPacket>(ros_recv_topic, 100, 
      std::bind(&SourcePacketRos::putPacket, this, std::placeholders::_1));
} 

void SourcePacketRos::putPacket(const yiliphoton_lidar::msg::YlplidarPacket::SharedPtr msg) const
{
  driver_ptr_->decodePacket(toAgMsg(*msg));
}

inline yiliphoton_lidar::msg::YlplidarPacket toRosMsg(const Packet& ag_msg, const std::string& frame_id)
{
  yiliphoton_lidar::msg::YlplidarPacket ros_msg;
  ros_msg.header.stamp.sec = (uint32_t)floor(ag_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((ag_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  //ros_msg.header.seq = ag_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.is_difop = ag_msg.is_difop;
  ros_msg.is_frame_begin = ag_msg.is_frame_begin;

  for (size_t i = 0; i < ag_msg.buf_.size(); i++)
  {
    ros_msg.data.emplace_back(ag_msg.buf_[i]);
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
  rclcpp::Publisher<yiliphoton_lidar::msg::YlplidarPacket>::SharedPtr pkt_pub_;
  std::string frame_id_;
};

inline void DestinationPacketRos::init(const YAML::Node& config)
{
  yamlRead<std::string>(config["ros"], 
      "ros_frame_id", frame_id_, "ylplidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", 
      ros_send_topic, "/ylplidar_packets");

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "ylplidar_packets_destination_" << node_index++;

  node_ptr_.reset(new rclcpp::Node(node_name.str()));
  pkt_pub_ = node_ptr_->create_publisher<yiliphoton_lidar::msg::YlplidarPacket>(ros_send_topic, 100);
}

inline void DestinationPacketRos::sendPacket(const Packet& msg)
{
  pkt_pub_->publish(toRosMsg(msg, frame_id_));
}

}  // namespace lidar
}  // namespace asensing

#endif  // ROS2_FOUND
