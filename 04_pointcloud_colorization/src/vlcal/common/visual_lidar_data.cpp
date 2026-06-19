#include <vlcal/common/visual_lidar_data.hpp>

#include <opencv2/opencv.hpp>
#include <vlcal/common/console_colors.hpp>

#include <glk/io/ply_io.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>          // For loading point cloud
#include <pcl/filters/voxel_grid.h>

namespace vlcal {

VisualLiDARData::VisualLiDARData(const std::string& data_path, const std::string& bag_name) {
  std::cout << "loading " << data_path + "/" + bag_name + ".(png|ply)" << std::endl;

  image = cv::imread(data_path + "/" + bag_name + ".png", 0);
  if (!image.data) {
    std::cerr << vlcal::console::bold_red << "warning: failed to load " << data_path + "/" + bag_name + ".png" << vlcal::console::reset << std::endl;
    abort();
  }

  auto ply = glk::load_ply(data_path + "/" + bag_name + ".ply");
  if (!ply) {
    std::cerr << vlcal::console::bold_red << "warning: failed to load " << data_path + "/" + bag_name + ".ply" << vlcal::console::reset << std::endl;
    abort();
  }

  points = std::make_shared<FrameCPU>(ply->vertices);
  points->add_intensities(ply->intensities);
}

VisualLiDARData::VisualLiDARData(const FrameData& keyframe) {
  // std::cout << "loading " << data_path + "/" + bag_name + ".(png|ply)" << std::endl;

  // image = cv::imread(data_path + "/" + bag_name + ".png", 0);
  // if (!image.data) {
  //   std::cerr << vlcal::console::bold_red << "warning: failed to load " << data_path + "/" + bag_name + ".png" << vlcal::console::reset << std::endl;
  //   abort();
  // }

  // auto ply = glk::load_ply(data_path + "/" + bag_name + ".ply");
  // if (!ply) {
  //   std::cerr << vlcal::console::bold_red << "warning: failed to load " << data_path + "/" + bag_name + ".ply" << vlcal::console::reset << std::endl;
  //   abort();
  // }

  // points = std::make_shared<FrameCPU>(ply->vertices);
  // points->add_intensities(ply->intensities);
  std::cout << "Loading " << keyframe.imagePath << std::endl;

  image = cv::imread(keyframe.imagePath, 0);
  if (!image.data) {
    std::cerr << vlcal::console::bold_red << "warning: failed to load " << keyframe.imagePath << vlcal::console::reset << std::endl;
    abort();
  }

  auto pcd = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(keyframe.culledPCDPath, *pcd) == -1)
  {
    std::cerr << vlcal::console::bold_red << "warning: failed to load " << keyframe.culledPCDPath << vlcal::console::reset << std::endl;
  }
  std::cout << "Loaded point cloud with " << pcd->points.size() << " points." << std::endl;

  // Downsample the point cloud to accelerate visualization in GUI
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_pcd(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(pcd);
  voxel_grid.setLeafSize(0.003f, 0.003f, 0.003f);  // Adjust the leaf size as needed
  voxel_grid.filter(*downsampled_pcd);

  std::cout << "Downsampled point cloud with " << downsampled_pcd->points.size() << " points." << std::endl;

  // Release the memory used by the original point cloud
  pcd.reset();

  points = std::make_shared<FrameCPU>(downsampled_pcd);

  // Extract intensities from point cloud pcd
  std::vector<double> intensities;
  intensities.reserve(downsampled_pcd->points.size());

  for (const auto& p : downsampled_pcd->points) {
      // std::cout << "p.intensity is:  "<< p.intensity << std::endl;
      intensities.push_back(p.intensity);
  }

  // Find the minimum and maximum intensity values
  auto [min_it, max_it] = std::minmax_element(intensities.begin(), intensities.end());
  float min_intensity = *min_it;
  float max_intensity = *max_it;

  // Normalize intensities to range [0, 255]
  std::transform(intensities.begin(), intensities.end(), intensities.begin(),
                 [min_intensity, max_intensity](float intensity) {
                     return 1.0f * (intensity - min_intensity) / (max_intensity - min_intensity);
                 });

  points->add_intensities(intensities);

}

VisualLiDARData::~VisualLiDARData() {}

}
