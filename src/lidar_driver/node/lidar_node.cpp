#include "manager/node_manager.hpp"

#include <lidar_sdk/common/version.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>

using namespace lidar_sdk::lidar;

static void sigHandler(int sig)
{
  LOG_MSG << "LiDAR-LiDAR-Driver is stopping....." << LOG_REND;
  ros::shutdown();
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);

  LOG_TITLE << "**********************************************" << LOG_REND;
  LOG_TITLE << "**  LiDAR Driver  v"
            << LIDAR_VERSION_MAJOR << "."
            << LIDAR_VERSION_MINOR << "."
            << LIDAR_VERSION_PATCH << "  **" << LOG_REND;
  LOG_TITLE << "**********************************************" << LOG_REND;

  ros::init(argc, argv, "lidar_node", ros::init_options::NoSigintHandler);

  std::string config_path = ros::package::getPath("lidar_driver") + "/config/config.yaml";

  ros::NodeHandle priv_nh("~");
  std::string path;
  priv_nh.param("config_path", path, std::string(""));
  if (!path.empty())
    config_path = path;

  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
  }
  catch (...)
  {
    LOG_ERROR << "Cannot load config file: " << config_path << LOG_REND;
    return -1;
  }

  std::shared_ptr<NodeManager> node_ptr = std::make_shared<NodeManager>();
  node_ptr->init(config);
  node_ptr->start();

  LOG_MSG << "LiDAR-LiDAR-Driver is running....." << LOG_REND;

  ros::spin();

  return 0;
}
