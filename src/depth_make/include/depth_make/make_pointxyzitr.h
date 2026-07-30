#define PCL_NO_PRECOMPILE 
#ifndef MAKE_POINTXYZITR_H
#define MAKE_POINTXYZITR_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
namespace pcl
{
struct PointXYZITR
{
    PCL_ADD_POINT4D;          
    float intensity;
    std::uint8_t ring;
    double timestamp;
    std::uint16_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;       
}
 
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZITR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint8_t, ring, ring)
                                  (double, timestamp, timestamp)
                                  (std::uint16_t, range, range)
                                  )
 
#endif
