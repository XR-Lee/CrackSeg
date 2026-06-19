#ifndef FRAMEDATA_HPP
#define FRAMEDATA_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

struct Pose
{
  double x, y, z, qw, qx, qy, qz;
};

struct Pose6D
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

// // Define a new point type that inherits from pcl::PointXYZRGB and adds a new float field
// struct EIGEN_ALIGN16 _PointXYZRGBMask // Forward declaration
// {
//   PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
//   // PCL_ADD_NORMAL4D;
//   // PCL_ADD_INTENSITY
//   PCL_ADD_RGB;
//   float segmentMask;
//   // double time;
// };

// struct EIGEN_ALIGN16 PointXYZRGBMask : public _PointXYZRGBMask
// {
//   inline constexpr PointXYZRGBMask (const _PointXYZRGBMask &p) :
//     PointXYZRGBMask {p.x, p.y, p.z, p.r, p.g, p.b, p.segmentMask} {}
  
//   inline constexpr PointXYZRGBMask (float _x, float _y, float _z,
//                                     std::int8_t _r, std::int8_t _g, std::int8_t _b, float _segmentMask) :
//     _PointXYZRGBMask {_x, _y, _z, _r, _g, _b, _segmentMask} {}

//   friend std::ostream& operator << (std::ostream& os, const PointXYZRGBMask& p)
//   {
//     os << "(" << "xyz: [" << p.x << "," << p.y << "," << p.z << "], ";
//     os << "rgb: [" << p.r << "," << p.g << "," << p.b << "], ";
//     os << "segmentMask: " << p.segmentMask << ")";
//     return (os);
//   }

//   PCL_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment for performance reasons
// };                  // Enforces SSE padding for correct memory alignment


// // Register the new point type with Point Cloud Library
// POINT_CLOUD_REGISTER_POINT_STRUCT(_PointXYZRGBMask,
//   (float, x, x)
//   (float, y, y)
//   (float, z, z)
//   (std::uint8_t, r, r)
//   (std::uint8_t, g, g)
//   (std::uint8_t, b, b)
//   (float, segmentMask, segmentMask)
// )
// POINT_CLOUD_REGISTER_POINT_WRAPPER(PointXYZRGBMask, _PointXYZRGBMask)

struct EIGEN_ALIGN16 PointXYZRGBMask
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_RGB;
  uint16_t segmentMask;

  PCL_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment for performance reasons
};                  // Enforces SSE padding for correct memory alignment

// Register the new point type with Point Cloud Library
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBMask,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  // (std::uint8_t, r, r)
  // (std::uint8_t, g, g)
  // (std::uint8_t, b, b)
  (float, rgb, rgb)
  (uint16_t, segmentMask, segmentMask)
)

class FrameData
{
public:
  using Ptr = std::shared_ptr<FrameData>;
  using ConstPtr = std::shared_ptr<const FrameData>;

  FrameData() {};
  // FrameData(const std::string &imagePath, double image_timestamp, const Pose &pose);
  FrameData(const std::string &imagePath, double timestamp, const Pose &pose)
      : imagePath(imagePath), imageTimestamp(timestamp), pose(pose) {
      visiblePoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  ~FrameData();

public:
  Pose pose;
  cv::Mat image; //raw image
  std::string imagePath;
  double imageTimestamp;

  cv::Mat segmentImage;
  std::string segmentImagePath;
  
  cv::Mat maskImage;
  std::string maskImagePath;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ptsInCamFrame;
  std::string culledPCDPath; // path to the culled pcd data, with no color, in camera coords

  pcl::PointCloud<pcl::PointXYZI>::Ptr visiblePoints;

  Eigen::Isometry3d initTLidarCamera;
  
  void addSegmentImage(const std::string &maskImagePath);
  // addImage(cv::Mat image, double timestamp);
  void addPts(const pcl::PointCloud<pcl::PointXYZI>::Ptr &points);
  void addManualOptimizedPose(Eigen::Isometry3d T_camera_lidar_optimized);
};

#endif // FRAMEDATA_HPP
