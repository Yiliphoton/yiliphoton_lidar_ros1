#include "lidar_sdk/api/lidar_driver.hpp"

#ifdef ENABLE_PCL_POINTCLOUD
#include "lidar_sdk/msg/pcl_point_cloud_msg.hpp"
#else
#include "lidar_sdk/msg/point_cloud_msg.hpp"
#endif

#include <signal.h>

//#define ORDERLY_EXIT
#define DEMO_DEBUG        0

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace lidar_sdk::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

LidarDriver<PointCloudMsg> driver;  // Declare the driver object
bool to_exit_process = false;

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver gets an free/unused point cloud message from the caller.
// @param msg  The free/unused point cloud message.
//
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed point cloud message to the caller. 
// @param msg  The stuffed point cloud message.
//
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
  stuffed_cloud_queue.push(msg);
}

//
// @brief exception callback function. The caller should register it to the lidar driver.
//        Via this function, the driver inform the caller that something happens.
// @param code The error code to represent the error/warning/information
//
void exceptionCallback(const Error& code)
{
  // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  LOG_WARNING << code.toString() << LOG_REND;
  if (code.error_code == ERRCODE_PCAPEXIT) {
    to_exit_process = true;
  }
}

void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the point cloud msg, even it is time-consuming.
    // LOG_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << ", timestamp: " << getTimeHost() << LOG_REND;
    LOG_DEBUG << "[" << msg->frame_id << "] Width = " << msg->width << ", Height = " << msg->height 
             << ", Total = " << msg->points.size() << ", is_dual = " << msg->is_dual 
             << ", timestamp: " << std::fixed << msg->timestamp << LOG_REND;

#if DEMO_DEBUG
    for (auto it = msg->points.begin(); it != msg->points.end(); it++)
    {
      std::cout << std::fixed << std::setprecision(3) 
                << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
                << std::endl;
    }
#endif

    free_cloud_queue.push(msg);
  }
}

static void sigHandler(int sig)
{
  LOG_MSG << "stop." << LOG_REND;
  driver.stop();
  exit(1);
}

int main(int argc, char* argv[])
{
  signal(SIGINT, sigHandler);  // bind ctrl+c signal with the sigHandler function

  LOG_TITLE << "------------------------------------------------------" << LOG_REND;
  LOG_TITLE << "            LiDAR_Driver Core Version: v" << getDriverVersion() << LOG_REND;
  LOG_TITLE << "------------------------------------------------------" << LOG_REND;

  DriverParam param;                                     // Create a parameter object
  param.input_type = InputType::PCAP_FILE;
  if (argv[1]) {
    param.input_param.pcap_path = argv[1];
  } else {
    param.input_param.pcap_path = "/home/rudy/workspace/FusionPosition/data/A2/2023-09-26-14-31-42.pcap";
  }
  // param.input_param.pcap_path = "/opt/data/A0-test.pcap";  // Set the pcap file directory
  param.input_param.pcap_repeat = false;                   // Set to no repeat
  param.input_param.msop_port = 51180;                     // Set the lidar msop port number, the default is 51180
  param.input_param.difop_port = 9988;                     // Set the lidar difop port number, the default is 9988
  param.lidar_type = LidarType::V192;                        // Set the lidar type. Make sure this type is correct
  param.decoder_param.wait_for_difop = false;
  param.decoder_param.use_lidar_clock = false;

  param.input_param.pcap_rate = 2.0;
  param.decoder_param.enable_fov_aligned = true;
  param.decoder_param.process_by_thread = true;

  param.print();

  // Register the point cloud callback functions
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback);
  driver.regExceptionCallback(exceptionCallback);  // Register the exception callback function
  if (!driver.init(param))                         // Call the init function
  {
    LOG_ERROR << "Driver Initialize Error..." << LOG_REND;
    return -1;
  }

  std::thread cloud_handle_thread = std::thread(processCloud);
#ifndef WIN32
  pthread_setname_np(cloud_handle_thread.native_handle(), "process_cloud");
#endif

  driver.start();  // The driver thread will start

  LOG_DEBUG << "LiDAR Lidar-Driver Linux pcap demo start......" << LOG_REND;

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();
#else
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (to_exit_process) {
      driver.stop();
      exit(0);
    }
  }
#endif

  return 0;
}
