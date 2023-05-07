#ifndef SFM_H_
#define SFM_H_

//ROS includes
#include <ros/ros.h>
#include <ros/console.h>

//all sensor includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

//message filter includes
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//pcl includes
#include <pcl/common/eigen.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

//Opencv includes
#include <cv_bridge/cv_bridge.h>

//Torch includes
#include <torch/torch.h>
#include <torch/script.h>

//Custom msg includes
#include <sfm/MapGraph.h>


//Main sfm class
class semanticSfm
{
private:
    std::string rgb_topic_;
    std::string depth_topic_;
    std::string intrensics_topic_;
    std::string extrensics_topic_;
    std::string model_path_;
    float leaf_size_;
    torch::jit::script::Module model; //Model
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> sfmSyncPolicy_;
    message_filters::Subscriber<sensor_msgs::Image> rgbImage_;
    message_filters::Subscriber<sensor_msgs::Image> depthMap_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cameraIntrensics_;
    message_filters::Subscriber<nav_msgs::Odometry> cameraExtrensics_;
    message_filters::Synchronizer<semanticSfm::sfmSyncPolicy_> sync_;
    ros::Publisher sfmCloud_;
    ros::Publisher sfmTrajectory_;
    ros::Publisher sfmMapGraph_;
    void sfmCallBack_(const sensor_msgs::Image::ConstPtr& rgb, const sensor_msgs::Image::ConstPtr& depthMap, const sensor_msgs::CameraInfo::ConstPtr& cameraIntrensics, const nav_msgs::Odometry::ConstPtr& cameraExtrensics);
    Eigen::Matrix4f poseToTransform_(nav_msgs::Odometry pose);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr transformCloud_(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& localCloud, Eigen::Matrix4f transform);
    pcl::PointXYZ projectDepthTo3D_(const cv::Mat depthMap, float w, float h, float cx, float cy, float fx, float fy);
    geometry_msgs::PoseStamped navToGeoStampMsg_(const nav_msgs::Odometry::ConstPtr& cameraExtrensics);
    cv::Mat forwardPass_(const cv::Mat rgbImage); //Network forward pass
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr globalCloud;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr downsampleCloud_(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& inputCloud, const float leafSize);
    geometry_msgs::Pose navToGeoMsg_(const nav_msgs::Odometry::ConstPtr& cameraExtrensics);
public:
    semanticSfm(ros::NodeHandle *nh);
    ~semanticSfm();
};

#endif
