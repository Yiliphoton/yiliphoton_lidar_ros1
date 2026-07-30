#include "manager/node_manager.hpp"

#include <ylp_driver/common/version.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>

using namespace asensing::lidar;

static void sigHandler(int sig)
{
  YLP_MSG << "Yiliphoton-LiDAR-Driver is stopping....." << YLP_REND;
  ros::shutdown();
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);

  YLP_TITLE << "**********************************************" << YLP_REND;
  YLP_TITLE << "**  Yiliphoton LiDAR Driver  v"
            << AGLIDAR_VERSION_MAJOR << "."
            << AGLIDAR_VERSION_MINOR << "."
            << AGLIDAR_VERSION_PATCH << "  **" << YLP_REND;
  YLP_TITLE << "**********************************************" << YLP_REND;

  ros::init(argc, argv, "yiliphoton_lidar_node", ros::init_options::NoSigintHandler);

  std::string config_path = ros::package::getPath("yiliphoton_lidar") + "/config/config.yaml";

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
    YLP_ERROR << "Cannot load config file: " << config_path << YLP_REND;
    return -1;
  }

  std::shared_ptr<NodeManager> node_ptr = std::make_shared<NodeManager>();
  node_ptr->init(config);
  node_ptr->start();

  YLP_MSG << "Yiliphoton-LiDAR-Driver is running....." << YLP_REND;

  ros::spin();

  return 0;
}
