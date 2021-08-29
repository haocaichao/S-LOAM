#ifndef SLOAM_HEADER_H
#define SLOAM_HEADER_H

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/voxel_grid.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/registration/icp.h>

//int N_SCAN_ROW=16;
int N_SCAN_ROW=64;

typedef struct {
    float value;
    int flag;
    int indexInRow;
} PointInfo;

/**
 * Velodyne点云结构，变量名XYZIRT是每个变量的首字母
*/
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D     // 位置
            PCL_ADD_INTENSITY;  // 激光点反射强度，也可以存点的索引
//    uint16_t ring;      // 扫描线
//    float time;         // 时间戳，记录相对于当前帧第一个激光点的时差，第一个点time=0
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;        // 内存16字节对齐，EIGEN SSE优化要求
// 注册为PCL点云格式
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
//(uint16_t, ring, ring)(float, time, time)
)

/**
 * 6D位姿点云结构定义
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                           (float, z, z) (float, intensity, intensity)
                                           (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                           (double, time, time))


#endif //SLOAM_HEADER_H
