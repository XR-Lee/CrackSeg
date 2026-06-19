#include "PointCloudProcessor.hpp"
#include "cloudSmooth.hpp"
#include "RGBCloud.hpp"
#include <pcl/io/pcd_io.h>          // For loading point cloud
#include <pcl/filters/voxel_grid.h> // Example for downsampling
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <fstream> // For reading odometry file
#include <filesystem>
#include <Eigen/Dense> // Add missing include statement for Eigen library
#include <tf/transform_datatypes.h>

#include "calibrate.cpp"
#include <vlcal/calib/view_culling.hpp>
#include <camera/create_camera.hpp>
#include <omp.h>
#include <vlcal/initial_guess_manual.hpp>

PointCloudProcessor::PointCloudProcessor(const std::string &pointCloudPath,
                                         const std::string &odometryPath,
                                         const std::string &imagesFolder,
                                         const std::string &maskImageFolder,
                                         const std::string &outputPath,
                                         const bool &enableMLS,
                                         const bool &enableNIDOptimize,
                                         const bool &enableInitialGuessManual)
    : pointCloudPath(pointCloudPath),
      odometryPath(odometryPath),
      imagesFolder(imagesFolder),
      maskImageFolder(maskImageFolder),
      outputPath(outputPath),
      enableMLS(enableMLS),
      enableNIDOptimize(enableNIDOptimize),
      enableInitialGuessManual(enableInitialGuessManual)

{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloudInCameraCoord.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloudInWorldWithRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloudInWorldWithRGBandMask.reset(new pcl::PointCloud<PointXYZRGBMask>());
    cloudInWorldWithMaskandMappedColor.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    frames.clear();
    selectedKeyframes.clear();

    R_lidar2cam << -0.99993085, -0.00561199, -0.0103344,
        0.01032389, 0.00189784, -0.99994491,
        0.0056313, -0.99998245, -0.00183977;
    t_lidar2cam << 0.071771636420221, -0.04934294727365431, -0.0677501086411397;
    // R_cam2imu;
    // t_cam2imu;
    K_camera = {4818.200388954926, 0.0, 2032.4178620390019,
                0.0, 4819.10345841615, 1535.1895959282901,
                0.0, 0.0, 1.0};
    D_camera = {0.003043514741045163, 0.06634739187544138, -0.000217681797407554, -0.0006654964142658197, 0};

    K_camera_coefficients = {K_camera[0], K_camera[4], K_camera[2], K_camera[5]};

    // Create an instance of the MLSParameters structure to hold the MLS parameters
    // bool enableMLS = false;
    // MLSParameters mlsParams;
    mlsParams.compute_normals = true;
    mlsParams.polynomial_order = 2;
    // mlsParams.search_radius = 0.03;
    mlsParams.search_radius = 0.03;
    mlsParams.sqr_gauss_param = 0.0009;
    mlsParams.num_threads = 30;

    mlsParams.slp_upsampling_radius = 0.05;
    mlsParams.slp_upsampling_stepsize = 0.01;
    mlsParams.rud_point_density = 50;
    // mlsParams.vgd_voxel_size = 0.001; // 0.001
    mlsParams.vgd_voxel_size = 0.001; // 0.001
    mlsParams.vgd_iterations = 4;
    // mlsParams.upsampling_enum = METHOD_VOXEL_GRID_DILATION;
    mlsParams.upsampling_enum = METHOD_VOXEL_GRID_DILATION;
    // mlsParams.upsampling_enum = None;

    mlsParams.sor_kmean_neighbour = 60;

    mlsParams.sor_std_dev = 0.7;

    // Check if maskImageFolder was provided; if not, set enableMaksSegmentation to false
    enableMaskSegmentation = !maskImageFolder.empty();
}

void PointCloudProcessor::loadPointCloud()
{
    // Create a bounding box that encompasses the odometry trajectory
    Eigen::Vector3d minPt(DBL_MAX, DBL_MAX, DBL_MAX);
    Eigen::Vector3d maxPt(DBL_MIN, DBL_MIN, DBL_MIN);
    for (const auto &frame : frames)
    {
        const Pose &pose = frame->pose;
        minPt = minPt.cwiseMin(Eigen::Vector3d(pose.x, pose.y, pose.z));
        maxPt = maxPt.cwiseMax(Eigen::Vector3d(pose.x, pose.y, pose.z));
    }

    // Inflate the bounding box a little bit if needed

    double padding = 2.0; // Add some padding to the bounding box
    minPt.array() -= padding;
    maxPt.array() += padding;

    // Load the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pointCloudPath, *originalCloud) == -1)
    {
        throw std::runtime_error("Couldn't read point cloud file.");
    }

    std::cout << "Start crop pcd..." << std::endl;

    // Crop the point cloud based on the bounding box
    pcl::CropBox<pcl::PointXYZI> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minPt.x(), minPt.y(), minPt.z(), 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxPt.x(), maxPt.y(), maxPt.z(), 1.0));
    boxFilter.setInputCloud(originalCloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZI>());
    boxFilter.filter(*croppedCloud);

    std::cout << "Loaded point cloud with " << originalCloud->points.size() << " points." << std::endl;
    std::cout << "Cropped point cloud with " << croppedCloud->points.size() << " points." << std::endl;

   // Generate the new file path for the cropped point cloud
    std::string croppedPointCloudPath = std::string(outputPath +  "scans-crop.pcd");;


    // Save the cropped point cloud
    pcl::io::savePCDFileASCII(croppedPointCloudPath, *croppedCloud);
    std::cout << "Cropped point cloud saved to: " << croppedPointCloudPath << std::endl;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudWithIntensity(new pcl::PointCloud<pcl::PointXYZINormal>());
    if (enableMLS)
    {
        CloudSmooth cloudSmooth(croppedPointCloudPath);
        cloudSmooth.initialize(mlsParams);
        cloudSmooth.process(cloudWithIntensity);
        pcl::copyPointCloud(*cloudWithIntensity, *cloud);
    }
    else
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pointCloudPath, *cloud) == -1)
        {
            throw std::runtime_error("Couldn't read point cloud file.");
        }
        std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;
    }
}

void PointCloudProcessor::applyNIDBasedPoseOptimization(std::vector<FrameData::Ptr> &keyframes)
{

    // Apply Multi-cost NID-based pose optimization
    vlcal::VisualLiDARCalibration calib(camera_model, K_camera_coefficients, D_camera, keyframes);
    calib.calibrate();
    // Eigen::Isometry3d T_camera_lidar_optimized;
    T_camera_lidar_optimized = calib.getOptimizedPose();
}

void PointCloudProcessor::applyInitialGuessManual(std::vector<FrameData::Ptr> &keyframes)
{
    for (auto &keyframe : keyframes)
    {
        // Apply manual initial guess to optimize the camera-lidar pose
        vlcal::InitialGuessManual init_guess(camera_model, K_camera_coefficients, D_camera, keyframe);

        // Main thread will be blocked until user exit init_guess by closing the GUI window
        init_guess.spin();
    }
}

void PointCloudProcessor::viewCullingAndSaveFilteredPcds(std::vector<FrameData::Ptr> &keyframes)
{
    // Save the culled point cloud into new pcd file, for the following NID-based pose optimization
    for (auto &keyframe : keyframes)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCameraPose(new pcl::PointCloud<pcl::PointXYZI>());

        // 1. Transform point cloud from world coordinates to camera pose coordinates
        Pose voPose = getPoseFromOdom(keyframe->pose);

        Eigen::Isometry3d t_c2w = Eigen::Isometry3d::Identity();
        t_c2w.translate(Eigen::Vector3d(voPose.x, voPose.y, voPose.z));
        t_c2w.rotate(Eigen::Quaterniond(voPose.qw, voPose.qx, voPose.qy, voPose.qz));

        Eigen::Affine3f transformation_w2c = t_c2w.inverse().cast<float>();
        Eigen::Affine3f transformation_c2w = t_c2w.cast<float>();
        Eigen::Affine3f transformation_c2w_optimized = transformation_c2w;

        pcl::transformPointCloud(*cloud, *cloudInCameraPose, transformation_w2c);

        // 2. Transform cloud from world coordinates to camera pose
        // , and apply view-culling to remove hidden points
        // , finally save these points to a new PCD file

        camera::GenericCameraBase::ConstPtr proj = camera::create_camera(camera_model, K_camera_coefficients, D_camera);
        vlcal::ViewCullingParams view_culling_params;
        // view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
        // std::cout << "before view_culling!" << std::endl;
        vlcal::ViewCulling view_culling(proj, {4096, 3000}, view_culling_params); // TODO: hardcode
        pcl::PointCloud<pcl::PointXYZI>::Ptr culledPCD = view_culling.cull(cloudInCameraPose, Eigen::Isometry3d::Identity());

        // cloudInCameraPose->clear();
        // pcl::copyPointCloud(*culled_points, *cloudInCameraPose);

        // 3. Save the culled point cloud into new pcd file, for the following NID-based pose optimization
        std::string culledPCDPath = std::string(outputPath + "filtered_pcd/" + std::to_string(keyframe->imageTimestamp) + "_beforeNID" + ".pcd");
        keyframe->culledPCDPath = culledPCDPath;
        pcl::PCDWriter pcd_writer;

        if (pcd_writer.writeASCII(culledPCDPath, *culledPCD) == -1)
        // if (pcd_writer.writeASCII(culledPCDPath, *cloudInCameraPose) == -1)
        {
            throw std::runtime_error("Couldn't save filtered point cloud to PCD file.");
        }
        std::cout << "Before NID optimization: view culling pcd saved to: " << culledPCDPath << ", the point size is " << culledPCD->size() << std::endl;
    }
}

/**
 * Applies field of view detection and hidden point removal to the given frame's point cloud.
 * This function transforms the point cloud from the world coordinate system to the camera coordinate system,
 * removes points that are occluded or hidden by other objects in the scene, and applies point cloud colorization
 * by projecting the point cloud onto the image frame and assigning the closest color to each point.
 *
 * @param frame The frame data containing the point cloud and camera pose.
 */
// void pcdColorization(std::vector<FrameData::Ptr> &keyframes);
// {
//     // 0. Init objects
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCameraPose(new pcl::PointCloud<pcl::PointXYZI>());

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanInBodyWithRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
//     pcl::PointCloud<PointXYZRGBMask>::Ptr scanInBodyWithRGBandMask(new pcl::PointCloud<PointXYZRGBMask>());

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanInWorldWithRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
//     pcl::PointCloud<PointXYZRGBMask>::Ptr scanInWorldWithRGBandMask(new pcl::PointCloud<PointXYZRGBMask>());

//     // 1. Transform point cloud from world coordinates to camera pose coordinates
//     Pose voPose = getPoseFromOdom(frame->pose);
//     Eigen::Isometry3d t_w2c = Eigen::Isometry3d::Identity();
//     Eigen::Isometry3d t_c2w = Eigen::Isometry3d::Identity();
//     Eigen::Quaterniond q_c2w(voPose.qw, voPose.qx, voPose.qy, voPose.qz); // qw, qx, qy, qz
//     Eigen::Vector3d trans_c2w(voPose.x, voPose.y, voPose.z);
//     t_c2w.translate(trans_c2w);
//     t_c2w.rotate(q_c2w);

//     t_w2c = t_c2w.inverse();
//     Eigen::Affine3f transformation_w2c = t_w2c.cast<float>();
//     Eigen::Affine3f transformation_c2w = t_c2w.cast<float>();
//     pcl::transformPointCloud(*cloud, *cloudInCameraPose, transformation_w2c);

//     // ////// 2. hidden point removal via open3d
//     // std::shared_ptr<open3d::geometry::PointCloud> o3d_cloud = ConvertPCLToOpen3D(cloudInCameraPose);
//     // std::shared_ptr<open3d::geometry::PointCloud> o3d_cloud_filtered = std::make_shared<open3d::geometry::PointCloud>();
//     // //// std::shared_ptr<open3d::geometry::TriangleMesh> o3d_cloud_filtered_mesh = std::make_shared<open3d::geometry::TriangleMesh>();

//     // //// Eigen::Vector3d camera_position = {voPose.x, voPose.y, voPose.z};
//     // Eigen::Vector3d camera_position = {0, 0, 0};
//     // double radius = 1000.0; // TODO: hardcode

//     // auto result = o3d_cloud->HiddenPointRemoval(camera_position, radius);
//     // auto o3d_cloud_filtered_mesh = std::get<0>(result);
//     // o3d_cloud_filtered = ConvertMeshToPointCloud(o3d_cloud_filtered_mesh);
//     // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered = ConvertOpen3DToPCL(o3d_cloud_filtered);

//     ////// 2.1 hidden point removal via NID and view_culling.cpp
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::copyPointCloud(*cloudInCameraPose, *pcl_cloud_filtered);

//     // 3. project 3d points to 2d images
//     const std::string camera_model = "pinhole";
//     std::vector<double> K_camera_coefficients = {K_camera[0], K_camera[4], K_camera[2], K_camera[5]};

//     // 3.1(optional): use NID metrics to optimize pose
//     Eigen::Affine3f transformation_c2w_optimized = transformation_c2w;
//     if (enableNIDOptimize)
//     {
//         vlcal::VisualLiDARCalibration calib(camera_model, K_camera_coefficients, D_camera, frame);
//         calib.calibrate(cloudInCameraPose, pcl_cloud_filtered);
//         Eigen::Isometry3d T_camera_lidar_optimized;
//         T_camera_lidar_optimized = calib.getOptimizedPose();

//         // Update camera pose with optimized camera-lidar extrinsic
//         Eigen::Isometry3d t_camere_world_optimized = t_c2w * T_camera_lidar_optimized;
//         Eigen::Affine3f transformation_c2w_optimized = t_camere_world_optimized.cast<float>();
//     }
//     else
//     {
//         camera::GenericCameraBase::ConstPtr proj = camera::create_camera(camera_model, K_camera_coefficients, D_camera);

//         vlcal::ViewCullingParams view_culling_params;
//         // view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
//         std::cout << "before view_culling!" << std::endl;
//         vlcal::ViewCulling view_culling(proj, {4096, 3000}, view_culling_params); // TODO: hardcode
//         pcl::PointCloud<pcl::PointXYZI>::Ptr culled_points = view_culling.cull(pcl_cloud_filtered, Eigen::Isometry3d::Identity());

//         pcl_cloud_filtered->clear();
//         for (const auto &point : culled_points->points)
//         {
//             pcl::PointXYZI transformed_point;
//             transformed_point.x = point.x;
//             transformed_point.y = point.y;
//             transformed_point.z = point.z;
//             transformed_point.intensity = point.intensity;

//             pcl_cloud_filtered->push_back(transformed_point);
//         }
//     }

//     generateColorMap(frame, pcl_cloud_filtered, scanInBodyWithRGB);

//     if (enableMaskSegmentation)
//     {
//         // 4. project 3d points to 2d segmentation mask images
//         generateSegmentMap(frame, scanInBodyWithRGB, scanInBodyWithRGBandMask);

//         // save the segmented point cloud
//         std::string filteredPointCloudPath = std::string(outputPath + "filtered_pcd/" + std::to_string(frame.imageTimestamp) + ".pcd");
//         pcl::PCDWriter pcd_writer;

//         if (pcd_writer.writeASCII(filteredPointCloudPath, *scanInBodyWithRGBandMask) == -1)
//         {
//             throw std::runtime_error("Couldn't save filtered point cloud to PCD file.");
//         }
//         std::cout << "Filtered point cloud saved to: " << filteredPointCloudPath << ", the point size is " << pcl_cloud_filtered->size() << std::endl;

//         // 5. Transforn colored scan into world frame, and combine them into one big colored cloud
//         if (enableNIDOptimize)
//         {
//             pcl::transformPointCloud(*scanInBodyWithRGBandMask, *scanInWorldWithRGBandMask, transformation_c2w_optimized);
//         }
//         else
//         {
//             pcl::transformPointCloud(*scanInBodyWithRGBandMask, *scanInWorldWithRGBandMask, transformation_c2w);
//         }

//         *cloudInWorldWithRGBandMask += *scanInWorldWithRGBandMask;
//     }
//     else
//     {
//         // 4. Save the colorized pointcloud to seperate PCD file
//         // visualizePointCloud(pcl_cloud_filtered);

//         std::string filteredPointCloudPath = std::string(outputPath + "filtered_pcd/" + std::to_string(frame.imageTimestamp) + ".pcd");
//         pcl::PCDWriter pcd_writer;

//         if (pcd_writer.writeASCII(filteredPointCloudPath, *scanInBodyWithRGB) == -1)
//         {
//             throw std::runtime_error("Couldn't save filtered point cloud to PCD file.");
//         }
//         std::cout << "Filtered point cloud saved to: " << filteredPointCloudPath << ", the point size is " << scanInBodyWithRGB->size() << std::endl;

//         // 5. Transforn colored scan into world frame, and combine them into one big colored cloud
//         if (enableNIDOptimize)
//         {
//             pcl::transformPointCloud(*scanInBodyWithRGB, *scanInWorldWithRGB, transformation_c2w_optimized);
//         }
//         else
//         {
//             pcl::transformPointCloud(*scanInBodyWithRGB, *scanInWorldWithRGB, transformation_c2w);
//         }

//         *cloudInWorldWithRGB += *scanInWorldWithRGB;
//     }
// }

/**
 * Applies field of view detection and hidden point removal to the given frame's point cloud.
 * This function transforms the point cloud from the world coordinate system to the camera coordinate system,
 * removes points that are occluded or hidden by other objects in the scene, and applies point cloud colorization
 * by projecting the point cloud onto the image frame and assigning the closest color to each point.
 *
 * @param frame The frame data containing the point cloud and camera pose.
 */
// void PointCloudProcessor::pcdColorization(std::vector<FrameData::Ptr> &keyframes)
// {
//     // 0. Init objects
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCameraPose(new pcl::PointCloud<pcl::PointXYZI>());

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanInBodyWithRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
//     pcl::PointCloud<PointXYZRGBMask>::Ptr scanInBodyWithRGBandMask(new pcl::PointCloud<PointXYZRGBMask>());

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanInWorldWithRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
//     pcl::PointCloud<PointXYZRGBMask>::Ptr scanInWorldWithRGBandMask(new pcl::PointCloud<PointXYZRGBMask>());

//     // 1. Transform point cloud from world coordinates to camera pose coordinates
//     for (auto keyframe : keyframes)
//     {
//         // Reset cloud and assign new objects at the beginning
//         scanInBodyWithRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
//         scanInBodyWithRGBandMask.reset(new pcl::PointCloud<PointXYZRGBMask>());
//         scanInWorldWithRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
//         scanInWorldWithRGBandMask.reset(new pcl::PointCloud<PointXYZRGBMask>());

//         Pose voPose = getPoseFromOdom(keyframe->pose);

//         Eigen::Isometry3d t_c2w = Eigen::Isometry3d::Identity();
//         t_c2w.translate(Eigen::Vector3d(voPose.x, voPose.y, voPose.z));
//         t_c2w.rotate(Eigen::Quaterniond(voPose.qw, voPose.qx, voPose.qy, voPose.qz));

//         Eigen::Affine3f transformation_w2c = t_c2w.inverse().cast<float>();
//         Eigen::Affine3f transformation_w2c_optimized = t_c2w.inverse().cast<float>();

//         Eigen::Affine3f transformation_c2w = t_c2w.cast<float>();
//         Eigen::Affine3f transformation_c2w_optimized = transformation_c2w;

//         if (enableNIDOptimize)
//         {
//             // transformation_c2w_optimized
//             Eigen::Isometry3d t_c2w_optimized = t_c2w * T_camera_lidar_optimized;
//             transformation_c2w_optimized = t_c2w_optimized.cast<float>();
//             transformation_w2c_optimized = transformation_c2w_optimized.inverse();
//         }

//         pcl::transformPointCloud(*cloud, *cloudInCameraPose, transformation_w2c_optimized);
//         camera::GenericCameraBase::ConstPtr proj = camera::create_camera(camera_model, K_camera_coefficients, D_camera);
//         vlcal::ViewCullingParams view_culling_params;
//         std::cout << "before view_culling!" << std::endl;
//         vlcal::ViewCulling view_culling(proj, {4096, 3000}, view_culling_params); // TODO: hardcode

//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCameraPoseCulled = view_culling.cull(cloudInCameraPose, Eigen::Isometry3d::Identity());
//         std::cout << " the point size of cloudInCameraPoseCulled is " << cloudInCameraPoseCulled->size() << std::endl;

//         generateColorMap(*keyframe, cloudInCameraPoseCulled, scanInBodyWithRGB);

//         if (enableMaskSegmentation)
//         {
//             // 4. project 3d points to 2d segmentation mask images
//             generateSegmentMap(*keyframe, scanInBodyWithRGB, scanInBodyWithRGBandMask);

//             // save the segmented point cloud
//             std::string filteredPointCloudPath = std::string(outputPath + "filtered_pcd/" + std::to_string(keyframe->imageTimestamp) + "_rgb-mask" + ".pcd");
//             pcl::PCDWriter pcd_writer;

//             if (pcd_writer.writeASCII(filteredPointCloudPath, *scanInBodyWithRGBandMask) == -1)
//             {
//                 throw std::runtime_error("Couldn't save filtered point cloud to PCD file.");
//             }
//             std::cout << "Filtered point cloud saved to: " << filteredPointCloudPath << ", the point size is " << scanInBodyWithRGBandMask->size() << std::endl;

//             // 5. Transforn colored scan into world frame, and combine them into one big colored cloud
//             pcl::transformPointCloud(*scanInBodyWithRGBandMask, *scanInWorldWithRGBandMask, transformation_c2w_optimized);
//             *cloudInWorldWithRGBandMask += *scanInWorldWithRGBandMask;
//         }
//         else
//         {
//             // 4. Save the colorized pointcloud to seperate PCD file
//             // visualizePointCloud(pcl_cloud_filtered);

//             std::string filteredPointCloudPath = std::string(outputPath + "filtered_pcd/" + std::to_string(keyframe->imageTimestamp) + "_rgb" + ".pcd");
//             pcl::PCDWriter pcd_writer;

//             if (pcd_writer.writeASCII(filteredPointCloudPath, *scanInBodyWithRGB) == -1)
//             {
//                 throw std::runtime_error("Couldn't save filtered point cloud to PCD file.");
//             }
//             std::cout << "Filtered point cloud saved to: " << filteredPointCloudPath << ", the point size is " << scanInBodyWithRGBandMask->size() << std::endl;

//             // 5. Transforn colored scan into world frame, and combine them into one big colored cloud
//             pcl::transformPointCloud(*scanInBodyWithRGB, *scanInWorldWithRGB, transformation_c2w_optimized);
//             *cloudInWorldWithRGB += *scanInWorldWithRGB;
//         }
//     }
//     saveColorizedPointCloud();
// }

void PointCloudProcessor::pcdColorizationAndSmooth(std::vector<FrameData::Ptr> &keyframes)
{
    RGBCloud rgbCloud;
    // rgbCloud.cloudNoColor = cloud;
    pcl::copyPointCloud(*cloud, *rgbCloud.cloudWithSmoothedColor);

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    const float epsilon = 1e-5; // define epsilon for point searching
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCameraPose(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::PointCloud<PointXYZRGBMask>::Ptr scanInBodyWithRGBandMask(new pcl::PointCloud<PointXYZRGBMask>());
    pcl::PointCloud<PointXYZRGBMask>::Ptr scanInWorldWithRGBandMask(new pcl::PointCloud<PointXYZRGBMask>());

    for (size_t keyframeIndex = 0; keyframeIndex < keyframes.size(); ++keyframeIndex)
    {
        scanInBodyWithRGBandMask.reset(new pcl::PointCloud<PointXYZRGBMask>());
        scanInWorldWithRGBandMask.reset(new pcl::PointCloud<PointXYZRGBMask>());

        auto &keyframe = keyframes[keyframeIndex];

        Pose voPose = getPoseFromOdom(keyframe->pose);
        Eigen::Isometry3d t_c2w = Eigen::Isometry3d::Identity();
        t_c2w.translate(Eigen::Vector3d(voPose.x, voPose.y, voPose.z));
        t_c2w.rotate(Eigen::Quaterniond(voPose.qw, voPose.qx, voPose.qy, voPose.qz));
        Eigen::Affine3f transformation_w2c = t_c2w.inverse().cast<float>();
        Eigen::Affine3f transformation_w2c_optimized = t_c2w.inverse().cast<float>();
        Eigen::Affine3f transformation_c2w = t_c2w.cast<float>();
        Eigen::Affine3f transformation_c2w_optimized = transformation_c2w;

        if (enableNIDOptimize)
        {
            // transformation_c2w_optimized
            Eigen::Isometry3d t_c2w_optimized = t_c2w * T_camera_lidar_optimized;
            transformation_c2w_optimized = t_c2w_optimized.cast<float>();
            transformation_w2c_optimized = transformation_c2w_optimized.inverse();
        }
        else if (enableInitialGuessManual)
        {
            // T_camera_lidar_optimized = (keyframe->initTLidarCamera).inverse();
            T_camera_lidar_optimized = keyframe->initTLidarCamera;
            std::cout << "Manually optimized T_camera_lidar_optimized is: " << T_camera_lidar_optimized.matrix() << std::endl;
            Eigen::Isometry3d t_c2w_optimized = t_c2w * T_camera_lidar_optimized;
            transformation_c2w_optimized = t_c2w_optimized.cast<float>();
            transformation_w2c_optimized = transformation_c2w_optimized.inverse();
        }

        pcl::transformPointCloud(*cloud, *cloudInCameraPose, transformation_w2c_optimized);
        camera::GenericCameraBase::ConstPtr proj = camera::create_camera(camera_model, K_camera_coefficients, D_camera);
        vlcal::ViewCullingParams view_culling_params;
        std::cout << "before view_culling!" << std::endl;
        vlcal::ViewCulling view_culling(proj, {4096, 3000}, view_culling_params); // TODO: hardcode

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCameraPoseCulled = view_culling.cull(cloudInCameraPose, Eigen::Isometry3d::Identity());
        std::cout << " the point size of cloudInCameraPoseCulled is " << cloudInCameraPoseCulled->size() << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        generateColorMap(*keyframe, cloudInCameraPoseCulled, coloredCloud);

        if (enableMaskSegmentation)
        {
            // project 3d points to 2d segmentation mask images
            generateSegmentMap(*keyframe, coloredCloud, scanInBodyWithRGBandMask);

            // save the segmented point cloud
            std::string filteredPointCloudPath = std::string(outputPath + "filtered_pcd/" + std::to_string(keyframe->imageTimestamp) + "_rgb-mask" + ".pcd");
            pcl::PCDWriter pcd_writer;

            if (pcd_writer.writeASCII(filteredPointCloudPath, *scanInBodyWithRGBandMask) == -1)
            {
                throw std::runtime_error("Couldn't save filtered point cloud to PCD file.");
            }
            std::cout << "Filtered point cloud saved to: " << filteredPointCloudPath << ", the point size is " << scanInBodyWithRGBandMask->size() << std::endl;

            // Transforn colored scan into world frame, and combine them into one big colored cloud
            pcl::transformPointCloud(*scanInBodyWithRGBandMask, *scanInWorldWithRGBandMask, transformation_c2w_optimized);
            *cloudInWorldWithRGBandMask += *scanInWorldWithRGBandMask;
        }

        //// Smooth color among multi-views
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloudInWorld(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*coloredCloud, *coloredCloudInWorld, transformation_c2w_optimized.cast<float>());

        for (size_t i = 0; i < coloredCloudInWorld->points.size(); ++i)
        {
            auto &ptColorInWrd = coloredCloudInWorld->points[i];

            // 将 ptColorInWrd 转换为 pcl::PointXYZI
            pcl::PointXYZI searchPoint;
            searchPoint.x = ptColorInWrd.x;
            searchPoint.y = ptColorInWrd.y;
            searchPoint.z = ptColorInWrd.z;

            // 使用 k-d 树进行近邻搜索
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            if (kdtree.radiusSearch(searchPoint, epsilon, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
                {
                    int pointIndex = pointIdxRadiusSearch[j];

                    // 将世界坐标系下的点转换到相机坐标系
                    Eigen::Vector4f ptWorld(ptColorInWrd.x, ptColorInWrd.y, ptColorInWrd.z, 1.0f);
                    Eigen::Vector4f ptCamera = transformation_c2w_optimized.inverse() * ptWorld;

                    pcl::PointXYZI pointInCamera;
                    pointInCamera.x = ptCamera.x();
                    pointInCamera.y = ptCamera.y();
                    pointInCamera.z = ptCamera.z();

                    float orientationScore = computeOrientationScore(pointInCamera, keyframe->pose);
                    float distanceScore = computeDistanceScore(pointInCamera);
                    float finalScore = (orientationScore + distanceScore) / 2.0;

                    RGBScore rgbScore(ptColorInWrd.r, ptColorInWrd.g, ptColorInWrd.b, orientationScore, distanceScore, finalScore);
                    rgbCloud.addPointData(pointIndex, rgbScore, keyframeIndex);
                }
            }
        }
    }
    smoothColors(rgbCloud);
    // smoothColorsWithLocalRegion(rgbCloud, 0.1); // 添加基于局部区域的颜色平滑
    removePointsWithNoColor(rgbCloud);
    if (enableMaskSegmentation) 
        saveColorizedPointCloud(rgbCloud, cloudInWorldWithRGBandMask);
    else saveColorizedPointCloud(rgbCloud);
}

void PointCloudProcessor::smoothColors(RGBCloud &rgbCloud)
{
    std::cout << "rgbCloud.cloudWithSmoothedColor->points size is: " << rgbCloud.cloudWithSmoothedColor->points.size() << std::endl;
    for (auto &point : rgbCloud.cloudWithSmoothedColor->points)
    {
        int pointIndex = &point - &rgbCloud.cloudWithSmoothedColor->points[0];
        PointData pointData = rgbCloud.getPointData(pointIndex);

        std::sort(pointData.rgbScores.begin(), pointData.rgbScores.end(), [](const RGBScore &a, const RGBScore &b)
                  { return a.finalScore > b.finalScore; });

        size_t M = std::min(pointData.rgbScores.size(), size_t(5)); // TODO: hardcode
        float totalScore = 0;
        float r = 0, g = 0, b = 0;
        for (size_t j = 0; j < M; ++j)
        {
            float score = pointData.rgbScores[j].finalScore;
            r += pointData.rgbScores[j].r * score;
            g += pointData.rgbScores[j].g * score;
            b += pointData.rgbScores[j].b * score;
            totalScore += score;
        }

        point.r = static_cast<uint8_t>(r / totalScore);
        point.g = static_cast<uint8_t>(g / totalScore);
        point.b = static_cast<uint8_t>(b / totalScore);
    }
}

// TODO: need optimizting memory usages
void PointCloudProcessor::smoothColorsWithLocalRegion(RGBCloud &rgbCloud, float radius)
{
    // 创建一个新的点云，用于存储平滑后的颜色
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothedColors(new pcl::PointCloud<pcl::PointXYZRGB>());
    smoothedColors->resize(rgbCloud.cloudWithSmoothedColor->points.size());

    // 创建八叉树用于邻域搜索
    float resolution = radius;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution);
    octree.setInputCloud(rgbCloud.cloudWithSmoothedColor);
    octree.addPointsFromInputCloud();

// 使用 OpenMP 并行化循环
#pragma omp parallel for
    for (size_t i = 0; i < rgbCloud.cloudWithSmoothedColor->points.size(); ++i)
    {
        auto &point = rgbCloud.cloudWithSmoothedColor->points[i];

        // 定义用于存储邻域搜索结果的向量
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // 进行radius搜索，找到当前点的邻域点
        if (octree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            // 初始化颜色累加值和总分数
            float totalScore = 0;
            float r = 0, g = 0, b = 0;

            // 遍历邻域点，进行颜色加权平均
            for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
            {
                auto &neighborPoint = rgbCloud.cloudWithSmoothedColor->points[pointIdxRadiusSearch[j]];
                float score = 1.0f / (1.0f + pointRadiusSquaredDistance[j]); // 简单的反距离权重

                // 累加颜色值和分数
                r += neighborPoint.r * score;
                g += neighborPoint.g * score;
                b += neighborPoint.b * score;
                totalScore += score;
            }

            // 设置平滑后的颜色值
            if (totalScore > 0)
            {
                smoothedColors->points[i].r = static_cast<uint8_t>(r / totalScore);
                smoothedColors->points[i].g = static_cast<uint8_t>(g / totalScore);
                smoothedColors->points[i].b = static_cast<uint8_t>(b / totalScore);
            }
            else
            {
                // 如果总分数为零，保留原颜色
                smoothedColors->points[i] = point;
            }
        }
        else
        {
            // 如果没有找到邻域点，保留原点的颜色值
            smoothedColors->points[i] = point;
        }
    }

    // 将平滑后的颜色应用到原始点云
    for (size_t i = 0; i < rgbCloud.cloudWithSmoothedColor->points.size(); ++i)
    {
        rgbCloud.cloudWithSmoothedColor->points[i].r = smoothedColors->points[i].r;
        rgbCloud.cloudWithSmoothedColor->points[i].g = smoothedColors->points[i].g;
        rgbCloud.cloudWithSmoothedColor->points[i].b = smoothedColors->points[i].b;
    }
}

void PointCloudProcessor::generateColorMap(const FrameData &frame,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr &pc,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_color)
{
    if (!pc || !pc_color)
    {
        throw std::runtime_error("PointCloud pointers are not initialized.");
        return;
    }

    std::cout << "Reading image from: " << frame.imagePath << std::endl;
    cv::Mat rgb = cv::imread(frame.imagePath);
    if (rgb.empty())
    {
        throw std::runtime_error("Failed to read image from: " + frame.imagePath);
        return;
    }
    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

    // 调整饱和度和亮度
    // TODO: hardcode
    float saturation_scale = 1.0; // 饱和度增加 0%
    float brightness_scale = 1.0; // 亮度增加 20%
    for (int y = 0; y < hsv.rows; y++)
    {
        for (int x = 0; x < hsv.cols; x++)
        {
            hsv.at<cv::Vec3b>(y, x)[1] = cv::saturate_cast<uchar>(hsv.at<cv::Vec3b>(y, x)[1] * saturation_scale);
            hsv.at<cv::Vec3b>(y, x)[2] = cv::saturate_cast<uchar>(hsv.at<cv::Vec3b>(y, x)[2] * brightness_scale);
        }
    }

    // 转换回 BGR 色彩空间
    cv::Mat adjusted_image;
    cv::cvtColor(hsv, adjusted_image, cv::COLOR_HSV2BGR);
    rgb = adjusted_image;

    for (int i = 0; i < pc->points.size(); i++)
    {
        // Eigen::Vector3d point_pc = {pc->points[i].x, pc->points[i].y, pc->points[i].z};
        Eigen::Vector3d point_camera = {pc->points[i].x, pc->points[i].y, pc->points[i].z};
        // Eigen::Vector3d point_camera = Rcl * point_pc + tcl;
        if (point_camera.z() > 0)
        {
            Eigen::Vector2d point_2d = (point_camera.head<2>() / point_camera.z()).eval();
            Eigen::Vector2d point_2d_dis = distort(point_2d);
            int u = static_cast<int>(K_camera[0] * point_2d_dis.x() + K_camera[2]);
            int v = static_cast<int>(K_camera[4] * point_2d_dis.y() + K_camera[5]);
            if (u >= 0 && u < rgb.cols && v >= 0 && v < rgb.rows)
            {
                pcl::PointXYZRGB point_rgb;
                point_rgb.x = point_camera.x();
                point_rgb.y = point_camera.y();
                point_rgb.z = point_camera.z();
                point_rgb.b = (rgb.at<cv::Vec3b>(v, u)[0]);
                point_rgb.g = (rgb.at<cv::Vec3b>(v, u)[1]);
                point_rgb.r = (rgb.at<cv::Vec3b>(v, u)[2]);
                pc_color->push_back(point_rgb);
            }
        }
    }
}

void PointCloudProcessor::generateSegmentMap(const FrameData &frame,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_color,
                                             pcl::PointCloud<PointXYZRGBMask>::Ptr &pc_color_segmented)
{

    std::cout << "Reading segment mask image from: " << frame.maskImagePath << std::endl;
    cv::Mat grayImg = cv::imread(frame.maskImagePath, cv::IMREAD_GRAYSCALE);
    if (grayImg.empty())
    {
        // throw std::runtime_error("Failed to read image from: " + frame.maskImagePath);
        std::cout << "Failed to read image from: " << frame.maskImagePath << std::endl;
        return;
    }

    for (int i = 0; i < pc_color->points.size(); i++)
    {
        // Eigen::Vector3d point_pc = {pc->points[i].x, pc->points[i].y, pc->points[i].z};
        Eigen::Vector3d point_camera = {pc_color->points[i].x, pc_color->points[i].y, pc_color->points[i].z};
        // Eigen::Vector3d point_camera = Rcl * point_pc + tcl;
        if (point_camera.z() > 0)
        {
            Eigen::Vector2d point_2d = (point_camera.head<2>() / point_camera.z()).eval();
            Eigen::Vector2d point_2d_dis = distort(point_2d);
            int u = static_cast<int>(K_camera[0] * point_2d_dis.x() + K_camera[2]);
            int v = static_cast<int>(K_camera[4] * point_2d_dis.y() + K_camera[5]);
            if (u >= 0 && u < grayImg.cols && v >= 0 && v < grayImg.rows)
            {
                PointXYZRGBMask point_rgb_segmented;
                point_rgb_segmented.x = point_camera.x();
                point_rgb_segmented.y = point_camera.y();
                point_rgb_segmented.z = point_camera.z();
                point_rgb_segmented.b = pc_color->points[i].b;
                point_rgb_segmented.g = pc_color->points[i].g;
                point_rgb_segmented.r = pc_color->points[i].r;
                // std::cout << grayImg.at<uchar>(v, u) << std::endl;
                point_rgb_segmented.segmentMask = grayImg.at<uchar>(v, u);
                // TODO: hardcode
                if (static_cast<int>(grayImg.at<uchar>(v, u)) == 255)
                {
                    point_rgb_segmented.r = 255;
                    point_rgb_segmented.g = 0;
                    point_rgb_segmented.b = 0;
                }   
                pc_color_segmented->push_back(point_rgb_segmented);
            }
        }
    }
}

void PointCloudProcessor::generateSegmentMapWithColor(pcl::PointCloud<PointXYZRGBMask>::Ptr &pc_color_segmented,
                                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_color)
{
    unsigned char r = 0, g = 255, b = 0; // Initialize to middle of the first color segment for safety

    // Jet colormap
    const int colormapLength = 4;
    float colormap[colormapLength][3] = {
        {0, 0, 0.5},
        {0, 0.5, 1},
        {0.5, 1, 0.5},
        {1, 0.5, 0}};

    for (size_t i = 0; i < pc_color_segmented->points.size(); i++)
    {
        pcl::PointXYZRGB point_rgb;
        point_rgb.x = pc_color_segmented->points[i].x;
        point_rgb.y = pc_color_segmented->points[i].y;
        point_rgb.z = pc_color_segmented->points[i].z;

        // Map the segmentMask value to RGB using Jet colormap
        float value = static_cast<float>(pc_color_segmented->points[i].segmentMask) / 255.0f; // Assuming segmentMask is normalized between 0 and 1
        value = std::min(std::max(value, 0.0f), 1.0f);                                        // Clamp value to range [0, 1] to avoid out-of-bounds access

        for (int j = 0; j < colormapLength - 1; j++)
        {
            float start = static_cast<float>(j) / (colormapLength - 1);
            float end = static_cast<float>(j + 1) / (colormapLength - 1);

            if (value >= start && value < end)
            {
                float ratio = (value - start) / (end - start);
                r = static_cast<uint8_t>(255 * ((colormap[j + 1][0] - colormap[j][0]) * ratio + colormap[j][0]));
                g = static_cast<uint8_t>(255 * ((colormap[j + 1][1] - colormap[j][1]) * ratio + colormap[j][1]));
                b = static_cast<uint8_t>(255 * ((colormap[j + 1][2] - colormap[j][2]) * ratio + colormap[j][2]));
                break;
            }
        }

        point_rgb.r = r;
        point_rgb.g = g;
        point_rgb.b = b;

        // std::cout << "R: " << static_cast<int>(r) << ", G: " << static_cast<int>(g) << ", B: " << static_cast<int>(b) << std::endl;
        pc_color->push_back(point_rgb);
    }
}

void PointCloudProcessor::saveColorizedPointCloud()
{
    if (enableMaskSegmentation)
    {
        if (cloudInWorldWithRGBandMask->size() > 0)
        {
            std::string cloudInWorldWithRGBDir(outputPath + "cloudInWorldWithRGBandMask.pcd");
            pcl::PCDWriter pcd_writer;

            if (pcd_writer.writeASCII(cloudInWorldWithRGBDir, *cloudInWorldWithRGBandMask) == -1)
            {
                throw std::runtime_error("Couldn't save colorized and segment colored point cloud.");
            }
            else
            {
                cout << "All colored and segment colored cloud saved to: " << cloudInWorldWithRGBandMask << endl;
            }

            generateSegmentMapWithColor(cloudInWorldWithRGBandMask, cloudInWorldWithMaskandMappedColor);
            if (cloudInWorldWithMaskandMappedColor->size() > 0)
            {
                std::string cloudInWorldWithMaskandMappedColorDir(outputPath + "cloudInWorldWithMaskAndMappedColor.pcd");
                pcl::PCDWriter pcd_writer_temp;
                pcd_writer_temp.writeASCII(cloudInWorldWithMaskandMappedColorDir, *cloudInWorldWithMaskandMappedColor);
            }
        }
    }
    else
    {
        if (cloudInWorldWithRGB->size() > 0)
        {
            std::string cloudInWorldWithRGBDir(outputPath + "cloudInWorldWithRGB.pcd");
            pcl::PCDWriter pcd_writer;

            if (pcd_writer.writeASCII(cloudInWorldWithRGBDir, *cloudInWorldWithRGB) == -1)
            {
                throw std::runtime_error("Couldn't save colorized point cloud.");
            }
            else
            {
                cout << "All colored cloud saved to: " << cloudInWorldWithRGB << endl;
            }
        }
    }
}

void PointCloudProcessor::saveColorizedPointCloud(const RGBCloud &rgbCloud)
{

    if (rgbCloud.cloudWithSmoothedColor->size() > 0)
    {
        std::string cloudInWorldWithRGBDir(outputPath + "cloudInWorldWithRGB.pcd");
        pcl::PCDWriter pcd_writer;

        if (pcd_writer.writeASCII(cloudInWorldWithRGBDir, *rgbCloud.cloudWithSmoothedColor) == -1)
        {
            throw std::runtime_error("Couldn't save colorized point cloud.");
        }
        else
        {
            std::cout << "All colored cloud saved to: " << cloudInWorldWithRGBDir << std::endl;
        }
    }

}

void PointCloudProcessor::saveColorizedPointCloud(const RGBCloud &rgbCloud, pcl::PointCloud<PointXYZRGBMask>::Ptr cloudInWorldWithRGBandMask)
{
    if (cloudInWorldWithRGBandMask->size() > 0)
    {
        std::string cloudInWorldWithRGBandMaskDir(outputPath + "cloudInWorldWithRGBandMask.pcd");
        pcl::PCDWriter pcd_writer;
        // TODO: fix it, replace cloudWithSmoothedColor
        if (pcd_writer.writeASCII(cloudInWorldWithRGBandMaskDir, *cloudInWorldWithRGBandMask) == -1)
        {
            throw std::runtime_error("Couldn't save colorized and segment colored point cloud.");
        }
        else
        {
            std::cout << "All colored and segment colored cloud saved to: " << cloudInWorldWithRGBandMaskDir << std::endl;
        }
    }

    if (rgbCloud.cloudWithSmoothedColor->size() > 0)
    {
        std::string cloudInWorldWithRGBDir(outputPath + "cloudInWorldWithRGB.pcd");
        pcl::PCDWriter pcd_writer;

        if (pcd_writer.writeASCII(cloudInWorldWithRGBDir, *rgbCloud.cloudWithSmoothedColor) == -1)
        {
            throw std::runtime_error("Couldn't save colorized point cloud.");
        }
        else
        {
            std::cout << "All colored cloud saved to: " << cloudInWorldWithRGBDir << std::endl;
        }
    }
}

void PointCloudProcessor::loadImagesAndOdometry()
{
    std::ifstream voFile(odometryPath);
    std::string line;

    while (getline(voFile, line))
    {
        std::istringstream iss(line);
        double timestamp, x, y, z, qw, qx, qy, qz;
        if (!(iss >> timestamp >> x >> y >> z >> qw >> qx >> qy >> qz))
        {
            break; // Error
        }
        Pose pose{x, y, z, qw, qx, qy, qz};

        // std::string imagePath = findImagePathForTimestamp(timestamp);
        std::string imagePath = imagesFolder + std::to_string(timestamp) + ".jpg";

        // Check if imagePath exists
        if (!std::filesystem::exists(imagePath))
        {
            continue; // Skip this image if it does not exist
        }

        if (enableMaskSegmentation)
        {
            std::string maskImagePath = maskImageFolder + std::to_string(timestamp) + ".png";
            if (!imagePath.empty())
            {
                FrameData frame(imagePath, timestamp, pose);
                frame.addSegmentImage(maskImagePath);
                frames.push_back(std::make_shared<FrameData>(frame));
            }
        }
        else
        {
            FrameData frame(imagePath, timestamp, pose);
            frames.emplace_back(std::make_shared<FrameData>(frame));
        }
    }
}

void PointCloudProcessor::process()
{
    loadImagesAndOdometry();

    loadPointCloud();

    generateResultStorageFolder();

    selectKeyframes();

    viewCullingAndSaveFilteredPcds(selectedKeyframes);

    if (enableNIDOptimize)
    {
        applyNIDBasedPoseOptimization(selectedKeyframes);
    }
    else if (enableInitialGuessManual)
    {
        applyInitialGuessManual(selectedKeyframes);
    }

    // pcdColorization(selectedKeyframes);
    // saveColorizedPointCloud();

    pcdColorizationAndSmooth(selectedKeyframes);
}

void PointCloudProcessor::generateResultStorageFolder()
{
    // create the output folder "filtered_pcd/" if not exists
    // , otherwise remove the original one and create new folder
    std::string filteredPcdFolderPath = outputPath + "filtered_pcd/";

    std::filesystem::path outputPcdDir(filteredPcdFolderPath);

    if (std::filesystem::exists(outputPcdDir))
    {
        std::filesystem::remove_all(outputPcdDir); // Delete the folder and all its contents
    }

    std::filesystem::create_directories(outputPcdDir); // Create the folder (and any necessary parent directories)
}

void PointCloudProcessor::selectKeyframes()
{
    bool isKeyframe = true;
    // Initialize keyframe identification variables
    FrameData::Ptr previousFrame = nullptr;
    // TODO: hardcode
    const double distThreshold = 0.1; // meter, 1
    const double angThreshold = 10.0; // degree. 25

    for (auto &frame : frames)
    {
        isKeyframe = markKeyframe(frame, previousFrame, distThreshold, angThreshold);
        if (isKeyframe)
        {
            std::cout << "\n Processing frame: " << frame->imagePath << std::endl;
            // Process each frame
            // applyFOVDetectionAndHiddenPointRemoval(frame);
            selectedKeyframes.push_back(frame);
            // colorizePoints();
            // smoothColors();
            isKeyframe = false;
            // previousFrame = const_cast<FrameData *>(&frame);
            previousFrame = frame;
        }
    }
}

void PointCloudProcessor::visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");

    // Set the background of the viewer to black
    viewer.setBackgroundColor(0, 0, 0);

    // Add the point cloud to the viewer with some unique ID
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "filteredCloud");

    // Add coordinate axes to the viewer
    viewer.addCoordinateSystem(1.0);

    // Set properties of the point cloud
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filteredCloud");

    // Main visualization loop
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);                                        // Update the viewer
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to reduce CPU usage
    }

    return;
}
