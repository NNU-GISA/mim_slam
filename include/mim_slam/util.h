#ifndef  _UTIL_H_
#define _UTIL_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/common.h>

#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>  

#include "mim_slam/Vector3.h"
#include "mim_slam/Quaternion.h"
#include "mim_slam/Pose6D.h"
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include "time.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.570796326794896619
#endif


#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329575)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

using namespace std;
using namespace alg;

/*imu或轮式里程计辅助，注：只能设置一个为true*/
extern const bool use_imu = false;
extern const int imu_queue_len = 200;
extern const string imu_topic = "/imu/data";

extern const bool use_wheel_odom = true;
extern const int wheel_odom_queue_len = 200;
extern const string wheel_odom_topic = "/odom";

//kitti数据集参数
extern const int N_SCAN = 40;  
extern const int Horizon_SCAN = 4000;  
extern const float ang_res_x = 0.09;  
extern const float ang_res_y = 0.65;  
extern const float ang_bottom = 24.0;  
extern const int groundScanInd = 32;   
extern const float range_max = 10000.0;

// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 2000;
// extern const float ang_res_x = 0.18;
// extern const float ang_res_y = 2;
// extern const float ang_bottom = 15.0;
// extern const int groundScanInd = 7;
// extern const float range_max = 10000.0;

extern const float sensorMountAngle = 0.0;
extern const float groundRemoveThre = 2.0;
extern const float segmentTheta = 1.0475;  // 1.0472
extern const int segmentMinNum = 30;    //30
extern const int segmentValidPointNum = 5;  //6
extern const int segmentValidLineNum = 3;   //3
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

extern const string fix_frame_id = "mim_map";
extern const string odom_frame_id = "mim_odom";
extern const string base_frame_id = "mim_base";

extern const float map_octree_resolution = 0.1;  //最终生成地图的分辨率
extern const float map_octree_radius = 0.6;        //近似近邻点时的搜索半径
extern const float ref_octree_resolution = 0.1;     //localizing时的参考点云的分辨率

//闭环检测
extern const bool check_for_loop_closure = false; 
extern const float translation_threshold = 0.5; //0.5m保留一帧关键帧
extern const int poses_before_reclosing = 500; //重闭环必须要在100次定位输出之外
extern const float max_tolerable_fitness = 0.5;
extern const float proximity_threshold = 5.0;
extern const int skip_recent_poses = 400;

#endif
