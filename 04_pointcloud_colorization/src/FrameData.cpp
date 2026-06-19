#include "FrameData.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

// FrameData::FrameData(const std::string &imagePath, double image_timestamp, const Pose &pose)
//     : pose(pose), imageTimestamp(image_timestamp)
// {

//     this->imagePath = imagePath;
//     this->imageTimestamp = image_timestamp;
//     this->pose = pose;
//     this->visiblePoints.reset(new pcl::PointCloud<pcl::PointXYZI>());

//     //    image = cv::imread(imagePath, cv::IMREAD_COLOR);
//     //    if (image.empty()) {
//     //        std::cerr << "Error loading image from path: " << imagePath << std::endl;
//     //        throw std::runtime_error("Failed to load image.");
//     //    }
// }

void FrameData::addSegmentImage(const std::string &maskImagePath)
{
    this->maskImagePath = maskImagePath;
    // this->maskImage = maskImage;
}

void FrameData::addPts(const pcl::PointCloud<pcl::PointXYZI>::Ptr &points){
    this->ptsInCamFrame = points;
}

void FrameData::addManualOptimizedPose(Eigen::Isometry3d T_camera_lidar_optimized){
    std::cout << "Adding manual optimized pose into keyframe!" << std::endl;
    this->initTLidarCamera = T_camera_lidar_optimized;
}


FrameData::~FrameData(){

}
