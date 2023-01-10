#include "sfm.h"


//sfm constructor
sfm::sfm(ros::NodeHandle *nh) : sync_(sfm::sfmSyncPolicy_(100), this->rgbImage_, this->depthMap_, this->cameraIntrensics_, this->cameraExtrensics_)
{
    nh->getParam("/structure_from_motion/rgb_topic", this->rgb_topic_);
    nh->getParam("/structure_from_motion/depth_topic", this->depth_topic_);
    nh->getParam("/structure_from_motion/camera_topic", this->intrensics_topic_);
    nh->getParam("/structure_from_motion/odom_topic", this->extrensics_topic_);
    this->sfmCloud_ = nh->advertise<sensor_msgs::PointCloud2>("sfm/cloud", 100);
    this->sfmTrajectory_ = nh->advertise<nav_msgs::Path>("sfm/trajectory", 100);
    this->rgbImage_.subscribe(*nh, this->rgb_topic_, 100);
    this->depthMap_.subscribe(*nh, this->depth_topic_, 100);
    this->cameraIntrensics_.subscribe(*nh, this->intrensics_topic_, 100);
    this->cameraExtrensics_.subscribe(*nh, this->extrensics_topic_, 100);
    this->sync_.registerCallback(std::bind(&sfm::sfmCallBack_, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

//sfm deconstructor
sfm::~sfm()
{
}

Eigen::Matrix4f sfm::poseToTransform_(nav_msgs::Odometry pose)
{
    Eigen::Matrix3f rotation = Eigen::Quaternionf(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z).toRotationMatrix();
    Eigen::Matrix4f transformation;
    transformation <<   rotation(0,0) , rotation(0,1) , rotation(0,2), pose.pose.pose.position.x,
                        rotation(1,0) , rotation(1,1) , rotation(1,2), pose.pose.pose.position.y,
                        rotation(2,0) , rotation(2,1) , rotation(2,2), pose.pose.pose.position.z,
                        0      ,      0        ,       0      ,           1           ;

    return transformation;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr sfm::transformCloud_(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr localCloud, Eigen::Matrix4f transform)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    output->resize(localCloud->size());
    pcl::transformPointCloud(*localCloud, *output, transform);
    return output;
}

pcl::PointXYZ sfm::projectDepthTo3D_(const cv::Mat depthMap, float w, float h, float cx, float cy, float fx, float fy)
{
    pcl::PointXYZ pt;
    float depth = depthMap.at<float>(h, w);
    if(depth > 0.0f)
    {
        // Fill in XYZ
        pt.x = (w - cx) * depth / fx;
        pt.y = (h - cy) * depth / fy;
        pt.z = depth;
    }
    else pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();

    return pt;
}

geometry_msgs::PoseStamped sfm::navToGeoMsg_(const nav_msgs::Odometry::ConstPtr& cameraExtrensics)
{
    geometry_msgs::PoseStamped currentOdom;
    currentOdom.header = cameraExtrensics->header;
    currentOdom.header.frame_id = "map";
    currentOdom.pose = cameraExtrensics->pose.pose;
    return currentOdom;
}

//main sfm function for point cloud generation
void sfm::sfmCallBack_(const sensor_msgs::Image::ConstPtr& rgb, const sensor_msgs::Image::ConstPtr& depthMap, const sensor_msgs::CameraInfo::ConstPtr& cameraIntrensics, const nav_msgs::Odometry::ConstPtr& cameraExtrensics)
{
    //Reading ros messages
    cv_bridge::CvImagePtr rgb_ptr, depthMap_ptr;
    try
    {
        rgb_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
        depthMap_ptr = cv_bridge::toCvCopy(depthMap);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //Image and depth Map
    cv::Mat rgbImage = rgb_ptr->image;
    cv::Mat depth = depthMap_ptr->image;
    
    //declaring point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr localCloudInWorldFrame(new pcl::PointCloud<pcl::PointXYZRGB>);
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //In camera frame (local cloud)
    int decimation = 4;
    localCloud->height = rgbImage.rows/decimation;
    localCloud->width  = rgbImage.cols/decimation;
    localCloud->is_dense = false;
    localCloud->resize(localCloud->height * localCloud->width);		
    for(int h = 0; h < rgbImage.rows && h/decimation < (int)localCloud->height ; h+=decimation)
    {
        for(int w = 0; w < rgbImage.cols && w/decimation < (int)localCloud->width ; w+=decimation)
        {
            pcl::PointXYZRGB & pt = localCloud->at((h/decimation)*localCloud->width + (w/decimation));

            int x = (w);
            int y = (h);
            pt.r = rgbImage.at<cv::Vec3b>(y, x)[2];
            pt.g = rgbImage.at<cv::Vec3b>(y, x)[1];
            pt.b = rgbImage.at<cv::Vec3b>(y, x)[0];
            pcl::PointXYZ ptXYZ = this->projectDepthTo3D_(depth, w, h, cameraIntrensics->K[2], cameraIntrensics->K[5], cameraIntrensics->K[0], cameraIntrensics->K[4]);
            if(pcl::isFinite(ptXYZ) && ptXYZ.z >= 0.0f && ptXYZ.z <= 4.0f) 
            {
                pt.x =  ptXYZ.z;
                pt.y = -ptXYZ.x;
                pt.z = -ptXYZ.y;
            }
            else pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
        }
    }

    //Removing NaNs from local cloud
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*localCloud, *localCloud, index);

    //Adding to global cloud using camera extrensics
    localCloudInWorldFrame = this->transformCloud_(localCloud, this->poseToTransform_(*cameraExtrensics));
    *globalCloud = *globalCloud + *localCloudInWorldFrame;

    //ROS messages
    sensor_msgs::PointCloud2 finalCloud;
    pcl::toROSMsg(*globalCloud, finalCloud);
    finalCloud.header = rgb->header;
    finalCloud.header.frame_id = "map";
    this->sfmCloud_.publish(finalCloud);
    static nav_msgs::Path trajectory;
    trajectory.header = rgb->header;
    trajectory.header.frame_id = "map";
    trajectory.poses.push_back(this->navToGeoMsg_(cameraExtrensics));
    this->sfmTrajectory_.publish(trajectory);
}


//main
int main(int argc, char** argv)
{
	ros::init(argc, argv, "sfm");
	ros::NodeHandle nh;
	sfm* sfm_ = new sfm(&nh);
	ros::spin();
	return 0;
}
