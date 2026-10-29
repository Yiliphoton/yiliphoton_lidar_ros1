
#pragma once

#include "lidar_sdk/msg/point_cloud_msg.hpp"

#ifdef POINT_TYPE_XYZIRTC
typedef PointCloudT<PointXYZIRTC> LidarPointCloudMsg;
#elif defined(POINT_TYPE_XYZIRT)
typedef PointCloudT<PointXYZIRT> LidarPointCloudMsg;
#else
typedef PointCloudT<PointXYZI> LidarPointCloudMsg;
#endif

