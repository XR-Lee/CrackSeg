#ifndef CLOUD_SMOOTH_INDIVIDUAL_H
#define CLOUD_SMOOTH_INDIVIDUAL_H

#include <iostream>
#include <mutex>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

enum UpsamplingMethodType
{
    METHOD_SAMPLE_LOCAL_PLANE,
    METHOD_RANDOM_UNIFORM_DENSITY,
    METHOD_VOXEL_GRID_DILATION,
    None
};

struct MLSParameters
{
    bool compute_normals;
    int polynomial_order;
    double search_radius;
    double sqr_gauss_param;
    int num_threads;
    double slp_upsampling_radius;
    double slp_upsampling_stepsize;
    int rud_point_density;
    double vgd_voxel_size;
    int vgd_iterations;
    int sor_kmean_neighbour;
    double sor_std_dev;
    UpsamplingMethodType upsampling_enum;
};

class CloudSmooth
{
public:
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAftSmooth;
    // Constructor that accepts the input file path
    CloudSmooth(const std::string &input_file_path);

    // Method to initialize the MLS parameters
    void initialize(const MLSParameters &params);

    // Method to process the input PCD file
    void process(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloudAftSmooth);

private:
    std::string input_file_path_;
    bool compute_normals_;
    int polynomial_order_;
    double search_radius_;
    double sqr_gauss_param_;
    int num_threads_;
    double slp_upsampling_radius_;
    double slp_upsampling_stepsize_;
    int rud_point_density_;
    double vgd_voxel_size_;
    int vgd_iterations_;
    int sor_kmean_neighbour_;
    double sor_std_dev_;
    std::mutex mutex_cloud;
    UpsamplingMethodType upsampling_enum_;
};

#endif // CLOUD_SMOOTH_INDIVIDUAL_H
