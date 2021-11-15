#include <pcl/io/io.h>
#include <pcl/point_types.h>

struct RsPointXYZIRT
{
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,(float,x,x)(float,y,y)
(float,z,z)(uint8_t,intensity,intensity)(uint16_t,ring,ring)(double,timestamp,timestamp))