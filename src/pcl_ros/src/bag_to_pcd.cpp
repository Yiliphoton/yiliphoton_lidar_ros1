#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <signal.h>

int frame_count = 0;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::string filename = "frame_" + std::to_string(frame_count++) + ".pcd";
  pcl::io::savePCDFileASCII(filename, cloud);
}

static void sigHandler(int) { ros::shutdown(); }

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);
  ros::init(argc, argv, "bag_to_pcd");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/lidar_points", 10, pointcloud_callback);
  ros::spin();
  return 0;
}
