#include "lidar_sdk/api/lidar_driver.hpp"
#include "lidar_sdk/msg/point_cloud_msg.hpp"

using namespace lidar_sdk::lidar;

typedef PointCloudT<PointXYZI> PointCloudMsg;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

bool checkKeywordExist(int argc, const char* const* argv, const char* str)
{
  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      return true;
    }
  }
  return false;
}

bool parseArgument(int argc, const char* const* argv, const char* str, std::string& val)
{
  int index = -1;

  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      index = i + 1;
    }
  }

  if (index > 0 && index < argc)
  {
    val = argv[index];
    return true;
  }

  return false;
}

void parseParam(int argc, char* argv[], DriverParam& param)
{
  std::string result_str;

  //
  // input param
  //
  parseArgument(argc, argv, "-pcap", param.input_param.pcap_path);
  if (param.input_param.pcap_path.empty())
  {
    param.input_type = InputType::ONLINE_LIDAR;
  }
  else
  {
    param.input_type = InputType::PCAP_FILE;
  }

  if (parseArgument(argc, argv, "-msop", result_str))
  {
    param.input_param.msop_port = std::stoi(result_str);
  }

  if (parseArgument(argc, argv, "-difop", result_str))
  {
    param.input_param.difop_port = std::stoi(result_str);
  }

  parseArgument(argc, argv, "-group", param.input_param.group_address);
  parseArgument(argc, argv, "-host", param.input_param.host_address);
 
  //
  // decoder param
  //
  if (parseArgument(argc, argv, "-type", result_str))
  {
    param.lidar_type = strToLidarType(result_str);
  }

  parseArgument(argc, argv, "-calib", param.decoder_param.calib_file);
  
  param.decoder_param.wait_for_difop = false;

  if (parseArgument(argc, argv, "-x", result_str))
  {
    param.decoder_param.transform_param.x = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-y", result_str))
  {
    param.decoder_param.transform_param.y = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-z", result_str))
  {
    param.decoder_param.transform_param.z = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-roll", result_str))
  {
    param.decoder_param.transform_param.roll = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-elevation", result_str))
  {
    param.decoder_param.transform_param.elevation = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-azimuth", result_str))
  {
    param.decoder_param.transform_param.azimuth = std::stof(result_str);
  }
}

void printHelpMenu()
{
  LOG_MSG << "Arguments: " << LOG_REND;
  LOG_MSG << "  -type   = LiDAR type (A0, A2)" << LOG_REND;
  LOG_MSG << "  -pcap   = The path of the pcap file, off-line mode if it is true, else online mode." << LOG_REND;
  LOG_MSG << "  -calib  = The calibration file name, if not specified, use default calibration file." << LOG_REND;
  LOG_MSG << "  -msop   = LiDAR msop port number,the default value is 51180" << LOG_REND;
  LOG_MSG << "  -difop  = LiDAR difop port number,the default value is 9988" << LOG_REND;
  LOG_MSG << "  -group  = LiDAR destination group address if multi-cast mode." << LOG_REND;
  LOG_MSG << "  -host   = Host address." << LOG_REND;
  LOG_MSG << "  -x      = Transformation parameter, unit: m " << LOG_REND;
  LOG_MSG << "  -y      = Transformation parameter, unit: m " << LOG_REND;
  LOG_MSG << "  -z      = Transformation parameter, unit: m " << LOG_REND;
  LOG_MSG << "  -roll   = Transformation parameter, unit: radian " << LOG_REND;
  LOG_MSG << "  -elevation  = Transformation parameter, unit: radian " << LOG_REND;
  LOG_MSG << "  -azimuth    = Transformation parameter, unit: radian " << LOG_REND;
  LOG_MSG << "" << LOG_REND;
  LOG_MSG << "Example: simple_csvsaver -type A0 -pcap /opt/data/A0-test.pcap" << LOG_REND;
}

void exceptionCallback(const Error& code)
{
  LOG_WARNING << code.toString() << LOG_REND;
}

std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
}

void saveCsv(const std::string &csv_path, const PointCloudMsg &cloud)
{
  LOG_MSG << "Save point cloud as " << csv_path << LOG_REND;

  std::ofstream os(csv_path, std::ios::out | std::ios::trunc);
  os << "Point ID,X,Y,Z,Intensity" << std::endl;

  for (size_t i = 0; i < cloud.points.size(); i++)
  {
    const PointXYZI& p = cloud.points[i];
    os << i+1 << ",";
    os << p.x << ",";
    os << p.y << ",";
    os << p.z << ",";
    os << (float)p.intensity << std::endl;
  }
}

bool to_exit_process = false;
void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    char csv_path[128];
    sprintf (csv_path, "%d.csv", msg->seq);
    saveCsv(csv_path, *msg);

    free_cloud_queue.push(msg);
  }
}

int main(int argc, char* argv[])
{
  LOG_TITLE << "------------------------------------------------------" << LOG_REND;
  LOG_TITLE << "            LiDAR_Driver CSV Saver Version: v" << getDriverVersion() << LOG_REND;
  LOG_TITLE << "------------------------------------------------------" << LOG_REND;

  if (argc < 2)
  {
    printHelpMenu();
    return 0;
  }

  if (checkKeywordExist(argc, argv, "-h") || checkKeywordExist(argc, argv, "--help"))
  {
    printHelpMenu();
    return 0;
  }

  std::thread cloud_handle_thread = std::thread(processCloud);
#ifndef WIN32
  pthread_setname_np(cloud_handle_thread.native_handle(), "process_cloud");
#endif

  DriverParam param;
  param.input_param.pcap_repeat = false;
  param.decoder_param.dense_points = false;
  param.decoder_param.enable_fov_aligned = false;

  parseParam(argc, argv, param);
  param.print();

  LidarDriver<PointCloudMsg> driver;
  driver.regExceptionCallback(exceptionCallback);
  driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback);
  if (!driver.init(param))
  {
    LOG_ERROR << "Driver Initialize Error..." << LOG_REND;
    return -1;
  }

  LOG_INFO << "LiDAR Lidar-Driver CSV Saver start......" << LOG_REND;

  driver.start();

  while (1)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();

  return 0;
}
