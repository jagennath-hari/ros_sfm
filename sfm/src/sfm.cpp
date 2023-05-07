#include "sfm.h"

//sfm constructor
semanticSfm::semanticSfm(ros::NodeHandle *nh) : sync_(semanticSfm::sfmSyncPolicy_(2), this->rgbImage_, this->depthMap_, this->cameraIntrensics_, this->cameraExtrensics_), globalCloud(new pcl::PointCloud<pcl::PointXYZRGBL>)
{
    nh->getParam("/structure_from_motion/rgb_topic", this->rgb_topic_);
    nh->getParam("/structure_from_motion/depth_topic", this->depth_topic_);
    nh->getParam("/structure_from_motion/camera_topic", this->intrensics_topic_);
    nh->getParam("/structure_from_motion/odom_topic", this->extrensics_topic_);
    nh->getParam("/structure_from_motion/model_path", this->model_path_);
    nh->getParam("/structure_from_motion/leaf_size", this->leaf_size_);
    this->model = torch::jit::load(this->model_path_);
    this->model.to(at::kCUDA);
    this->sfmCloud_ = nh->advertise<sensor_msgs::PointCloud2>("sfm/cloud", 1);
    this->sfmTrajectory_ = nh->advertise<nav_msgs::Path>("sfm/trajectory", 1);
    this->sfmMapGraph_ = nh->advertise<sfm::MapGraph>("sfm/MapGraph", 1);
    this->rgbImage_.subscribe(*nh, this->rgb_topic_, 2);
    this->depthMap_.subscribe(*nh, this->depth_topic_, 2);
    this->cameraIntrensics_.subscribe(*nh, this->intrensics_topic_, 2);
    this->cameraExtrensics_.subscribe(*nh, this->extrensics_topic_, 2);
    this->sync_.registerCallback(std::bind(&semanticSfm::sfmCallBack_, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

//sfm deconstructor
semanticSfm::~semanticSfm()
{
}

Eigen::Matrix4f semanticSfm::poseToTransform_(nav_msgs::Odometry pose)
{
    Eigen::Matrix3f rotation = Eigen::Quaternionf(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z).toRotationMatrix();
    Eigen::Matrix4f transformation;
    transformation <<   rotation(0,0) , rotation(0,1) , rotation(0,2), pose.pose.pose.position.x,
                        rotation(1,0) , rotation(1,1) , rotation(1,2), pose.pose.pose.position.y,
                        rotation(2,0) , rotation(2,1) , rotation(2,2), pose.pose.pose.position.z,
                        0      ,      0        ,       0      ,           1           ;

    return transformation;
}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr semanticSfm::transformCloud_(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& localCloud, Eigen::Matrix4f transform)
{
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBL>);
    output->resize(localCloud->size());
    pcl::transformPointCloud(*localCloud, *output, transform);
    return output;
}

pcl::PointXYZ semanticSfm::projectDepthTo3D_(const cv::Mat depthMap, float w, float h, float cx, float cy, float fx, float fy)
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

geometry_msgs::PoseStamped semanticSfm::navToGeoStampMsg_(const nav_msgs::Odometry::ConstPtr& cameraExtrensics)
{
    geometry_msgs::PoseStamped currentOdom;
    currentOdom.header = cameraExtrensics->header;
    currentOdom.header.frame_id = "map";
    currentOdom.pose = cameraExtrensics->pose.pose;
    return currentOdom;
}

cv::Mat semanticSfm::forwardPass_(const cv::Mat rgbImage)
{
    cv::Mat img_float;
    rgbImage.convertTo(img_float, CV_32FC3, 1.0f );
    torch::Tensor x = torch::from_blob(img_float.data, {img_float.rows, img_float.cols, 3});
    x = x.to(at::kFloat).div(255).unsqueeze(0);
    x = x.permute({ 0, 3, 1, 2 });
    std::vector<double> norm_mean = {0.486,0.456,0.406};
    std::vector<double> norm_std = {0.229,0.224,0.225};
    x = torch::data::transforms::Normalize<>(norm_mean, norm_std)(x);
    std::vector<torch::jit::IValue> x2;
    x2.push_back(x.cuda());
    auto Output2 = this->model.forward(x2).toTensor();
    auto output2_c = Output2.to(torch::kCPU);
    auto m = torch::nn::Threshold(0.6, 0);
    output2_c = m(output2_c);
    auto b = torch::argmax(output2_c, 1)[0];
    b = b.toType(torch::kFloat);

    cv::Mat raw_segmentated = cv::Mat(cv::Size(b.size(1), b.size(0)), CV_32FC(1), b.data_ptr());
    cv::Mat segmentated;
    raw_segmentated.convertTo(segmentated, CV_8UC1);

    return segmentated;
}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr semanticSfm::downsampleCloud_(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& inputCloud, const float leafSize)
{
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr output_filtered(new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::VoxelGrid<pcl::PointXYZRGBL> filter;
	filter.setDownsampleAllData(true);
    filter.setLeafSize(leafSize, leafSize, leafSize);
    filter.setInputCloud(inputCloud);             
    filter.filter(*output_filtered);

    return output_filtered;
}

geometry_msgs::Pose semanticSfm::navToGeoMsg_(const nav_msgs::Odometry::ConstPtr& cameraExtrensics)
{
    geometry_msgs::Pose currentOdom = cameraExtrensics->pose.pose;
    return currentOdom;
}

//main sfm function for point cloud generation
void semanticSfm::sfmCallBack_(const sensor_msgs::Image::ConstPtr& rgb, const sensor_msgs::Image::ConstPtr& depthMap, const sensor_msgs::CameraInfo::ConstPtr& cameraIntrensics, const nav_msgs::Odometry::ConstPtr& cameraExtrensics)
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
    //Image, depth Map and segmentation
    cv::Mat rgbImage = rgb_ptr->image;
    cv::Mat depth = depthMap_ptr->image;
    cv::Mat segmenation = this->forwardPass_(rgbImage);
    
    //declaring point clouds
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);

    //In camera frame (local cloud)
    int decimation = 4;
    localCloud->height = rgbImage.rows/decimation;
    localCloud->width  = rgbImage.cols/decimation;
    localCloud->is_dense = false;
    localCloud->resize(localCloud->height * localCloud->width);	

    //Perform threading to speed up	(lock_guard + mutex + thread) (four - eight threads running from each section of image)
    for(int h = 0; h < rgbImage.rows && h/decimation < (int)localCloud->height ; h+=decimation)
    {
        for(int w = 0; w < rgbImage.cols && w/decimation < (int)localCloud->width ; w+=decimation)
        {
            pcl::PointXYZRGBL & pt = localCloud->at((h/decimation)*localCloud->width + (w/decimation));
            pt.r = rgbImage.at<cv::Vec3b>(h, w)[2];
            pt.g = rgbImage.at<cv::Vec3b>(h, w)[1];
            pt.b = rgbImage.at<cv::Vec3b>(h, w)[0];
            pt.label = segmenation.at<uchar>(h, w);
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
    *(this->globalCloud) = *(this->globalCloud) + *(this->downsampleCloud_(this->transformCloud_(localCloud, this->poseToTransform_(*cameraExtrensics)), this->leaf_size_));

    //ROS messages
    sensor_msgs::PointCloud2 finalCloud;
    pcl::toROSMsg(*(this->globalCloud), finalCloud);
    finalCloud.header = rgb->header;
    finalCloud.header.frame_id = "map";
    this->sfmCloud_.publish(finalCloud);
    static nav_msgs::Path trajectory;
    trajectory.header = rgb->header;
    trajectory.header.frame_id = "map";
    trajectory.poses.push_back(this->navToGeoStampMsg_(cameraExtrensics));
    this->sfmTrajectory_.publish(trajectory);

    //MapGraph custom Msg
    static sfm::MapGraph mapGraphMsg;
    mapGraphMsg.header = rgb->header;
    mapGraphMsg.header.frame_id = "map";
    mapGraphMsg.poses.push_back(this->navToGeoMsg_(cameraExtrensics));
    mapGraphMsg.posesId.push_back(mapGraphMsg.poses.size() - 1);
    this->sfmMapGraph_.publish(mapGraphMsg);
}


//main
int main(int argc, char** argv)
{
	ros::init(argc, argv, "sfm");
	ros::NodeHandle nh;
	semanticSfm* semanticSfm_ = new semanticSfm(&nh);
	ros::MultiThreadedSpinner spinner(12);
    spinner.spin();

	return 0;
}
