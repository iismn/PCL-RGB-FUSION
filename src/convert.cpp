#include "VL_FUSE/VIS_LIDAR_FUSE.h"

VIS_LIDAR_FUSE::VIS_LIDAR_FUSE()
{
  VL_TF_INFO_CHECK = false;
  V_INFO_CHECK = false;
  BUSY = false;
  IMG_FRAME = "";
}

void VIS_LIDAR_FUSE::INITIALIZE()
{
  ros::NodeHandle P_NH("~");
  tf::TransformListener tf_listner;
  TF_SUB = &tf_listner;

  INITIALIZE_ROS(P_NH);

  ros::MultiThreadedSpinner spinner(8);
  spinner.spin();
}

void VIS_LIDAR_FUSE::INITIALIZE_ROS(ros::NodeHandle &P_NH)
{
  //get params
  std::string points_src, image_src, camera_info_src, fused_topic_str = "/points_fused";
  std::string name_space_str = ros::this_node::getNamespace();

  P_NH.param<std::string>("points_src", points_src, "/points_raw");
  P_NH.param<std::string>("image_src", image_src, "/image_rectified");
  P_NH.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
  P_NH.param<float>("PHAROS_AGV/SENSOR_CONFIG/VISION/Logi_BRIO_4K/Fx", Fx, 1.144810406181205e+03);
  P_NH.param<float>("PHAROS_AGV/SENSOR_CONFIG/VISION/Logi_BRIO_4K/Fy", Fy, 1.144669963436012e+03);
  P_NH.param<float>("PHAROS_AGV/SENSOR_CONFIG/VISION/Logi_BRIO_4K/Cx", Cx, 9.662573529698977e+02);
  P_NH.param<float>("PHAROS_AGV/SENSOR_CONFIG/VISION/Logi_BRIO_4K/Cy", Cy, 5.430427209650604e+02);
  P_NH.param<float>("PHAROS_AGV/SENSOR_CONFIG/VISION/Logi_BRIO_4K/imgHeight", image_H, 1.144810406181205e+03);
  P_NH.param<float>("PHAROS_AGV/SENSOR_CONFIG/VISION/Logi_BRIO_4K/imgWidth", image_W, 1.144810406181205e+03);

  cout << Fx << " " << Fy << " " << Cx << " " << Cy << " " << endl;

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      name_space_str.erase(name_space_str.begin());
    }
    image_src = name_space_str + image_src;
    fused_topic_str = name_space_str + fused_topic_str;
    camera_info_src = name_space_str + camera_info_src;
  }

  V_INTRINSIC_SUB = P_NH.subscribe(camera_info_src, 1, &VIS_LIDAR_FUSE::Intrinsics_Callback, this);
  PCL_SUB = P_NH.subscribe(image_src, 1, &VIS_LIDAR_FUSE::Image_Callback, this);
  IMG_SUB = P_NH.subscribe(points_src, 1, &VIS_LIDAR_FUSE::Cloud_Callback, this);

  VL_FUSE_PUB = NH.advertise<sensor_msgs::PointCloud2>(fused_topic_str, 1);

}


void VIS_LIDAR_FUSE::Image_Callback(const sensor_msgs::CompressedImage::ConstPtr &image_msg)
{
  if (!V_INFO_CHECK)
  {
    ROS_INFO("[WARN] INTRINSIC TOPIC NONE / USING DEFAULT", __APP_NAME__);

    V_SIZE.height = image_H;
    V_SIZE.width = image_W;

    V_FX = Fx;
    V_FY = Fy;
    V_CX = Cx;
    V_CY = Cy;

    // Eigen::MatrixXd PRJEC_MATRIX(3, 4);
    PRJEC_MATRIX = Eigen::MatrixXf::Zero(3, 4);
    PRJEC_MATRIX(0,0) = V_FX;
    PRJEC_MATRIX(1,1) = V_FY;
    PRJEC_MATRIX(0,2) = V_CX;
    PRJEC_MATRIX(1,2) = V_CY;
    PRJEC_MATRIX(2,2) = 1.0;

    V_INFO_CHECK = true;
  }
  if (BUSY)
    return;

  cv::Mat in_image = cv::imdecode(cv::Mat(image_msg->data),1);

  cv::Mat undistorted_image;
  CURRENT_FRAME = in_image;

  IMG_FRAME = image_msg->header.frame_id;
  V_SIZE.height = CURRENT_FRAME.rows;
  V_SIZE.width = CURRENT_FRAME.cols;
}

void VIS_LIDAR_FUSE::Cloud_Callback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg)
{
  if (CURRENT_FRAME.empty() || IMG_FRAME == "")
  {
    ROS_INFO("[WARN] CAM TO LiDAR(VLP16) TF LISTENING", __APP_NAME__);
    return;
  }
  if (!VL_TF_INFO_CHECK)
  {
    VL_TF = TF_SE3(IMG_FRAME, in_cloud_msg->header.frame_id);
  }
  if (!V_INFO_CHECK)
  {
    ROS_INFO("[WARN] INTRINSIC TOPIC NONE / USING DEFAULT", __APP_NAME__);

    V_SIZE.height = image_H;
    V_SIZE.width = image_W;

    V_FX = Fx;
    V_FY = Fy;
    V_CX = Cx;
    V_CY = Cy;

    // Eigen::MatrixXd PRJEC_MATRIX(3, 4);
    PRJEC_MATRIX = Eigen::MatrixXf::Zero(3, 4);
    PRJEC_MATRIX(0,0) = V_FX;
    PRJEC_MATRIX(1,1) = V_FY;
    PRJEC_MATRIX(0,2) = V_CX;
    PRJEC_MATRIX(1,2) = V_CY;
    PRJEC_MATRIX(2,2) = 1.0;

    V_INFO_CHECK = true;
  }
  if (!V_INFO_CHECK || !VL_TF_INFO_CHECK){
    ROS_INFO("[WARN] CAM TO LiDAR(VLP16) TF LISTENING", __APP_NAME__);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_IN(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*in_cloud_msg, *PCL_IN);

  tf::transformTFToEigen(VL_TF,CAM_LIDAR_AF3);
  SE3_CAM_LIDAR = CAM_LIDAR_AF3.matrix();
  pcl::transformPointCloud(*PCL_IN, *PCL_IN, SE3_CAM_LIDAR);

  ROI_FILTER.setInputCloud (PCL_IN);
  ROI_FILTER.setFilterFieldName ("z");
  ROI_FILTER.setFilterLimits (0.0, 100);
  ROI_FILTER.filter (*PCL_IN);

  Eigen::MatrixXf EIGEN_PCL = PCL_IN->getMatrixXfMap();
  Eigen::MatrixXf PRJEC_PCL = Eigen::MatrixXf::Zero(6,EIGEN_PCL.cols());
  PRJEC_PCL.block(3,0,3,EIGEN_PCL.cols()) = EIGEN_PCL.block(0,0,3,EIGEN_PCL.cols());

  EIGEN_PCL = PRJEC_MATRIX*EIGEN_PCL;
  EIGEN_PCL= EIGEN_PCL.array().rowwise() / EIGEN_PCL.row(2).array();
  PRJEC_PCL.block(0,0,3,EIGEN_PCL.cols()) = EIGEN_PCL;

  Eigen::ArrayXf Indices_U;
  Eigen::ArrayXf Indices_V;
  Eigen::ArrayXf Indices_FNL;
  Indices_U = (EIGEN_PCL.row(0).array()>0 && EIGEN_PCL.row(0).array()<1920).cast<float>();
  Indices_V = (EIGEN_PCL.row(1).array()>0 && EIGEN_PCL.row(1).array()<1080).cast<float>();
  Indices_FNL = (Indices_U.cast<bool>() && Indices_V.cast<bool>()).cast<float>();


  // PRJEC_PCL = PRJEC_PCL.select(Indices_FNL);
  CURRENT_FRAME_DATA = CURRENT_FRAME.data;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  out_cloud->points.clear();

  for(int i = 0;i<PRJEC_PCL.cols();i++){
    if(Indices_FNL(i)==1){
      pcl::PointXYZRGB VL_FUSE_PCL_TEMP;
      PixelCoord_U = floor(PRJEC_PCL(0,i));
      PixelCoord_V = floor(PRJEC_PCL(1,i));
      PixelInterP_a = PRJEC_PCL(0,i) - PixelCoord_U;
      PixelInterP_b = PRJEC_PCL(1,i) - PixelCoord_V;

      VL_FUSE_PCL_TEMP.x = PRJEC_PCL(3,i);
      VL_FUSE_PCL_TEMP.y = PRJEC_PCL(4,i);
      VL_FUSE_PCL_TEMP.z = PRJEC_PCL(5,i);

      int index = PixelCoord_V * CURRENT_FRAME.cols * 3 + PixelCoord_U * 3;

      VL_FUSE_PCL_TEMP.r =
      (1-PixelInterP_a)*(1-PixelInterP_b)*CURRENT_FRAME_DATA[index + 2]
      +(PixelInterP_a)*(1-PixelInterP_b)*CURRENT_FRAME_DATA[index + 2]
      +(PixelInterP_a)*(PixelInterP_b)*CURRENT_FRAME_DATA[index + 2]
      +(1-PixelInterP_a)*(PixelInterP_b)*CURRENT_FRAME_DATA[index + 2];

      VL_FUSE_PCL_TEMP.g =
      (1-PixelInterP_a)*(1-PixelInterP_b)*CURRENT_FRAME_DATA[index + 1]
      +(PixelInterP_a)*(1-PixelInterP_b)*CURRENT_FRAME_DATA[index + 1]
      +(PixelInterP_a)*(PixelInterP_b)*CURRENT_FRAME_DATA[index + 1]
      +(1-PixelInterP_a)*(PixelInterP_b)*CURRENT_FRAME_DATA[index + 1];

      VL_FUSE_PCL_TEMP.b =
      (1-PixelInterP_a)*(1-PixelInterP_b)*CURRENT_FRAME_DATA[index]
      +(PixelInterP_a)*(1-PixelInterP_b)*CURRENT_FRAME_DATA[index]
      +(PixelInterP_a)*(PixelInterP_b)*CURRENT_FRAME_DATA[index]
      +(1-PixelInterP_a)*(PixelInterP_b)*CURRENT_FRAME_DATA[index];

      cout << "R : " << VL_FUSE_PCL_TEMP.r+0 << " G : " << VL_FUSE_PCL_TEMP.g+0 << " B : " << VL_FUSE_PCL_TEMP.b+0 << endl;
      if ((VL_FUSE_PCL_TEMP.r == 192 && VL_FUSE_PCL_TEMP.g == 129 && VL_FUSE_PCL_TEMP.b == 0)||(VL_FUSE_PCL_TEMP.r == 128 && VL_FUSE_PCL_TEMP.g == 128 && VL_FUSE_PCL_TEMP.b == 128)){
        out_cloud->points.push_back(VL_FUSE_PCL_TEMP);
      }

    }
  }

  Eigen::Matrix4d SE3_CAM_LIDAR_INV = SE3_CAM_LIDAR.inverse();
  pcl::transformPointCloud(*out_cloud, *out_cloud, SE3_CAM_LIDAR_INV);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*out_cloud, cloud_msg);
  cloud_msg.header = in_cloud_msg->header;
  VL_FUSE_PUB.publish(cloud_msg);

}

void VIS_LIDAR_FUSE::Intrinsics_Callback(const sensor_msgs::CameraInfoConstPtr &cam_info_msg)
{
  V_SIZE.height = cam_info_msg->height;
  V_SIZE.width = cam_info_msg->width;

  V_FX = static_cast<float>(cam_info_msg->P[0]);
  V_FY = static_cast<float>(cam_info_msg->P[5]);
  V_CX = static_cast<float>(cam_info_msg->P[2]);
  V_CY = static_cast<float>(cam_info_msg->P[6]);

  // Eigen::MatrixXd PRJEC_MATRIX(3, 4);
  PRJEC_MATRIX = Eigen::MatrixXf::Zero(3, 4);
  PRJEC_MATRIX(0,0) = V_FX;
  PRJEC_MATRIX(1,1) = V_FY;
  PRJEC_MATRIX(0,2) = V_CX;
  PRJEC_MATRIX(1,2) = V_CY;
  PRJEC_MATRIX(2,2) = 1.0;

  V_INTRINSIC_SUB.shutdown();
  V_INFO_CHECK = true;
}

pcl::PointXYZ VIS_LIDAR_FUSE::TF_PCL(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
  tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
  tf::Vector3 tf_point_t = in_transform * tf_point;
  return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

tf::StampedTransform VIS_LIDAR_FUSE::TF_SE3(const std::string &in_target_frame, const std::string &in_source_frame)
{
  tf::StampedTransform transform;

  VL_TF_INFO_CHECK = false;
  try
  {
    TF_SUB->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
    VL_TF_INFO_CHECK = true;
    ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
  }

  return transform;
}
