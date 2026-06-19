#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>

#include <boost/filesystem.hpp>
#include <boost/asio/thread_pool.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <omp.h>

#include "cloudSmooth.hpp"

CloudSmooth::CloudSmooth(const std::string &input_file_path)
    : input_file_path_(input_file_path),
      compute_normals_(true),
      polynomial_order_(2),
      search_radius_(0.03),
      sqr_gauss_param_(0.0009),
      num_threads_(12),
      slp_upsampling_radius_(0.05),
      slp_upsampling_stepsize_(0.01),
      rud_point_density_(50),
      vgd_voxel_size_(0.001),
      vgd_iterations_(5),
      sor_kmean_neighbour_(6),
      sor_std_dev_(0.3),
      upsampling_enum_(METHOD_VOXEL_GRID_DILATION) {}

void CloudSmooth::initialize(const MLSParameters &params)
{
    // Initialize MLS parameters using the external params structure
    compute_normals_ = params.compute_normals;
    polynomial_order_ = params.polynomial_order;
    search_radius_ = params.search_radius;
    sqr_gauss_param_ = params.sqr_gauss_param;
    num_threads_ = params.num_threads;
    slp_upsampling_radius_ = params.slp_upsampling_radius;
    slp_upsampling_stepsize_ = params.slp_upsampling_stepsize;
    rud_point_density_ = params.rud_point_density;
    vgd_voxel_size_ = params.vgd_voxel_size;
    vgd_iterations_ = params.vgd_iterations;
    sor_kmean_neighbour_ = params.sor_kmean_neighbour;
    sor_std_dev_ = params.sor_std_dev;
    upsampling_enum_ = params.upsampling_enum;

    std::cout << "\n\n==============================\n"
              << "MLS Parameters:"
              << "\n    Compute Normals: " << compute_normals_
              << "\n    Polynomial Order: " << polynomial_order_
              << "\n    Search Radius: " << search_radius_
              << "\n    Squared Gaussian Parameter: " << sqr_gauss_param_
              << "\n    Number of Threads: " << num_threads_
              << "\n    SLP Upsampling Radius: " << slp_upsampling_radius_
              << "\n    SLP Upsampling Step Size: " << slp_upsampling_stepsize_
              << "\n    RUD Point Density: " << rud_point_density_
              << "\n    VGD Voxel Size: " << vgd_voxel_size_
              << "\n    VGD Iterations: " << vgd_iterations_
              << "\n    SOR K-Mean Neighbour: " << sor_kmean_neighbour_
              << "\n    SOR Standard Deviation: " << sor_std_dev_
              << "\n    Upsampling Method: " << upsampling_enum_
              << "\n==============================\n\n"
              << std::endl;
}

void CloudSmooth::process(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloudAftSmooth)
{
    // Initialize the cloud and tree
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormal(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAftSor(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointNormal> mls;

    //// Load the input PCD file
    std::cout << "\n\n==============================\n"
              << "MLS: Load PCD file: " << input_file_path_
              << "\n==============================\n"
              << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file_path_, *cloud) == -1)
    {
        std::cerr << "Couldn't read file " << input_file_path_ << std::endl;
        return;
    }
    std::cout << "Success read file " << input_file_path_ << std::endl;

#ifdef _OPENMP
    // Process the PCD file
    auto start = std::chrono::high_resolution_clock::now();
    size_t initial_point_count = cloud->points.size();

    //// Apply statistical outlier removal
    std::cout << "\n==============================\n" 
              << "MLS: Apply 1st statistical outlier removal before MLS" 
              << std::endl;

    pcl::copyPointCloud(*cloud, *cloudWithNormal);
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sorOriginCloud;
    // sorOriginCloud.setInputCloud(*cloud);
    sorOriginCloud.setInputCloud(pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>(*cloudWithNormal));
    sorOriginCloud.setMeanK(sor_kmean_neighbour_);
    sorOriginCloud.setStddevMulThresh(sor_std_dev_);
    sorOriginCloud.filter(*cloudWithNormal);
    std::cout << "\n cloudWithNormal point count: " << cloudWithNormal->size() << std::endl;

    pcl::copyPointCloud(*cloudWithNormal, *cloudAftSor);

#ifdef _OPENMP
    // Perform MLS smoothing
    std::cout << "====== MLS: Perform MLS smoothing " << std::endl;
    std::cout << "\ncloudAftSor point count: " << cloudAftSor->size() << std::endl;
    mls.setInputCloud(cloudAftSor);
    mls.setComputeNormals(compute_normals_);
    mls.setPolynomialOrder(polynomial_order_);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(search_radius_);
    mls.setSqrGaussParam(sqr_gauss_param_);
    mls.setNumberOfThreads(num_threads_);

    // Choose the upsampling method
    switch (upsampling_enum_)
    {
    case METHOD_SAMPLE_LOCAL_PLANE:
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius(slp_upsampling_radius_);
        mls.setUpsamplingStepSize(slp_upsampling_stepsize_);
        break;
    case METHOD_RANDOM_UNIFORM_DENSITY:
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
        mls.setPointDensity(rud_point_density_);
        break;
    case METHOD_VOXEL_GRID_DILATION:
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointNormal>::VOXEL_GRID_DILATION);
        mls.setDilationVoxelSize(vgd_voxel_size_);
        mls.setDilationIterations(vgd_iterations_);
        break;
    default:
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointNormal>::NONE);
        break;
    }

    mls.process(mls_points);
    std::cout << "\nmls_points point count: " << mls_points.size() << std::endl;
#endif

    // Apply statistical outlier removal
    std::cout << "====== MLS: Apply 2nd statistical outlier removal after MLS " << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sorAfterMLS;
    sorAfterMLS.setInputCloud(mls_points.makeShared());
    sorAfterMLS.setMeanK(sor_kmean_neighbour_);
    sorAfterMLS.setStddevMulThresh(sor_std_dev_);
    sorAfterMLS.filter(mls_points);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

#endif

    // Log the processing information
    std::cout << "\n==============================\n"
              << " MLS Summary: Processed file: " << input_file_path_
              << "\nInitial point count: " << initial_point_count
              << "\nFinal point count: " << mls_points.size()
              << "\nElapsed time: " << elapsed.count() << " s"
              << "\n==============================\n"
              << std::endl;

    // Save the output file
    std::string output_file = boost::filesystem::path(input_file_path_).stem().string() + "_mls.pcd";
    pcl::io::savePCDFile(output_file, mls_points);
    // *cloudAftSmooth = mls_points;
    pcl::copyPointCloud(mls_points, *cloudAftSmooth);
    // return cloudAftSmooth;
}
