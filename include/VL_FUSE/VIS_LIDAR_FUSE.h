
#ifndef VL_FUSE_H
#define VL_FUSE_H

#define __APP_NAME__ "VIS_LIDAR_FUSE"

// STD Library
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <cmath>

// ROS Library
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// ROS Library : VISUAL INFO
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

// ROS Library : PCL INFO
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// ROS Library : MSG
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Eigen>
#include <Eigen/Core>


namespace std {
  template <>
  class hash< cv::Point >{
  public :
    size_t operator()(const cv::Point &pixel_cloud ) const
    {
      return hash<std::string>()( std::to_string(pixel_cloud.x) + "|" + std::to_string(pixel_cloud.y) );
    }
  };
};

using namespace std;
using namespace cv;

class VIS_LIDAR_FUSE
{
  ros::NodeHandle                     NH;
  
  ros::Publisher                      VL_FUSE_PUB;
  ros::Subscriber                     V_INTRINSIC_SUB;

  tf::TransformListener*              TF_SUB;
  tf::StampedTransform                VL_TF;

  cv::Size                            V_SIZE;
  cv::Mat                             K;
  cv::Mat                             DIST_CEFF;
  cv::Mat                             CURRENT_FRAME;

  std::string                         IMG_FRAME;
  uchar*                              CURRENT_FRAME_DATA;
  bool                                BUSY;
  bool                                V_INFO_CHECK;
  bool                                VL_TF_INFO_CHECK;

  Eigen::Affine3d                     CAM_LIDAR_AF3;
  Eigen::Matrix4d                     SE3_CAM_LIDAR;
  Eigen::MatrixXf                     PRJEC_MATRIX;

  float                               V_FX, V_FY, V_CX, V_CY;
  float                               image_H, image_W, Fx, Fy, Cx, Cy;
  int                                 PixelCoord_U;
  int                                 PixelCoord_V;
  float                               PixelInterP_a;
  float                               PixelInterP_b;

  pcl::PointCloud<pcl::PointXYZRGB>   VL_FUSE_PCL;
  pcl::PassThrough<pcl::PointXYZ>     ROI_FILTER;


  ros::Subscriber                     PCL_SUB;
  ros::Subscriber                     IMG_SUB;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> LIDAR_VIS_SYNC;
  message_filters::Synchronizer<LIDAR_VIS_SYNC>              *LIDAR_VIS_SYNCRONIZER_;

  void Intrinsics_Callback(const sensor_msgs::CameraInfoConstPtr &cam_info_msg);
  void Image_Callback(const sensor_msgs::CompressedImage::ConstPtr &in_image_msg);
  void Cloud_Callback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg);
  void INITIALIZE_ROS(ros::NodeHandle &in_private_handle);

  tf::StampedTransform TF_SE3(const std::string &in_target_frame, const std::string &in_source_frame);
  pcl::PointXYZ TF_PCL(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform);


public:
  void INITIALIZE();
  VIS_LIDAR_FUSE();
};

#endif //VL_FUSE_H
