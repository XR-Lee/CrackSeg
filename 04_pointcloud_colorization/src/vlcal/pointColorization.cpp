#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <vector>
#include <algorithm>

struct ScorePointPair
{
    float score;
    pcl::PointXYZRGB point;
};

float computeOrientationScore(const Eigen::Vector3d &point_to_cam, const Eigen::Vector3d &cam_plane_normal)
{
    return std::cos(point_to_cam.dot(cam_plane_normal) / (point_to_cam.norm() * cam_plane_normal.norm()));
}

float computeDistanceScore(float distance, float idealDistance, float sigma)
{
    return std::exp(-std::pow(distance - idealDistance, 2) / (2 * std::pow(sigma, 2)));
}

std::vector<ScorePointPair> selectBestViews(const std::vector<FrameData::Ptr> &keyframes, const pcl::PointXYZRGB &point, int N, float idealDistance, float sigma)
{
    std::vector<ScorePointPair> scores;

    for (const auto &keyframe : keyframes)
    {
        Pose voPose = getPoseFromOdom(keyframe->pose);
        Eigen::Vector3d point_to_cam(voPose.x - point.x, voPose.y - point.y, voPose.z - point.z);
        Eigen::Vector3d cam_plane_normal(0, 0, 1); // Assuming camera plane normal is in Z direction

        float distance = point_to_cam.norm();
        float orientation_score = computeOrientationScore(point_to_cam, cam_plane_normal);
        float distance_score = computeDistanceScore(distance, idealDistance, sigma);
        float final_score = orientation_score + distance_score;

        scores.push_back({final_score, point});
    }

    // Sort scores to select top N views
    std::sort(scores.begin(), scores.end(), [](const ScorePointPair &a, const ScorePointPair &b)
              { return a.score > b.score; });

    // Return top N views
    return std::vector<ScorePointPair>(scores.begin(), scores.begin() + std::min(N, static_cast<int>(scores.size())));
}

void smoothPointColor(pcl::PointXYZRGB &point, const std::vector<ScorePointPair> &bestViews)
{
    float total_weight = 0.0f;
    float r = 0, g = 0, b = 0;

    for (const auto &view : bestViews)
    {
        float weight = view.score;
        total_weight += weight;
        r += view.point.r * weight;
        g += view.point.g * weight;
        b += view.point.b * weight;
    }

    if (total_weight > 0)
    {
        point.r = static_cast<uint8_t>(r / total_weight);
        point.g = static_cast<uint8_t>(g / total_weight);
        point.b = static_cast<uint8_t>(b / total_weight);
    }
}

void PointCloudProcessor::pcdColorization(std::vector<FrameData::Ptr> &keyframes, int N, float idealDistance = 2.0f, float sigma = 1.0f)
{
    // 0. Init objects
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCameraPose(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanInBodyWithRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<PointXYZRGBMask>::Ptr scanInBodyWithRGBandMask(new pcl::PointCloud<PointXYZRGBMask>());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanInWorldWithRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<PointXYZRGBMask>::Ptr scanInWorldWithRGBandMask(new pcl::PointCloud<PointXYZRGBMask>());

    // 1. Transform point cloud from world coordinates to camera pose coordinates
    for (auto keyframe : keyframes)
    {
        // Reset cloud and assign new objects at the beginning
        scanInBodyWithRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        scanInBodyWithRGBandMask.reset(new pcl::PointCloud<PointXYZRGBMask>());
        scanInWorldWithRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        scanInWorldWithRGBandMask.reset(new pcl::PointCloud<PointXYZRGBMask>());

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

        pcl::transformPointCloud(*cloud, *cloudInCameraPose, transformation_w2c_optimized);

        // Filter and cull points
        camera::GenericCameraBase::ConstPtr proj = camera::create_camera(camera_model, K_camera_coefficients, D_camera);
        vlcal::ViewCullingParams view_culling_params;
        vlcal::ViewCulling view_culling(proj, {4096, 3000}, view_culling_params); // TODO: hardcode

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInCameraPoseCulled = view_culling.cull(cloudInCameraPose, Eigen::Isometry3d::Identity());
        std::cout << " the point size of cloudInCameraPoseCulled is " << cloudInCameraPoseCulled->size() << std::endl;

        generateColorMap(*keyframe, cloudInCameraPoseCulled, scanInBodyWithRGB);

        // New logic to select the best top N views for each point and smooth the color
        for (auto &point : scanInBodyWithRGB->points)
        {
            std::vector<ScorePointPair> bestViews = selectBestViews(keyframes, point, N, idealDistance, sigma);
            smoothPointColor(point, bestViews);
        }

        if (enableMaskSegmentation)
        {
            generateSegmentMap(*keyframe, scanInBodyWithRGB, scanInBodyWithRGBandMask);

            // save the segmented point cloud
            std::string filteredPointCloudPath = std::string(outputPath + "filtered_pcd/" + std::to_string(keyframe->imageTimestamp) + "_colored" + ".pcd");
            pcl::PCDWriter pcd_writer;

            if (pcd_writer.writeASCII(filteredPointCloudPath, *scanInBodyWithRGBandMask) == -1)
            {
                throw std::runtime_error("Couldn't save filtered point cloud to PCD file.");
            }
            std::cout << "Filtered point cloud saved to: " << filteredPointCloudPath << ", the point size is " << scanInBodyWithRGBandMask->size() << std::endl;

            // Transform colored scan into world frame, and combine them into one big colored cloud
            pcl::transformPointCloud(*scanInBodyWithRGBandMask, *scanInWorldWithRGBandMask, transformation_c2w_optimized);
            *cloudInWorldWithRGBandMask += *scanInWorldWithRGBandMask;
        }
        else
        {
            // Save the colorized pointcloud to separate PCD file
            std::string filteredPointCloudPath = std::string(outputPath + "filtered_pcd/" + std::to_string(keyframe->imageTimestamp) + "_colored" + ".pcd");
            pcl::PCDWriter pcd_writer;

            if (pcd_writer.writeASCII(filteredPointCloudPath, *scanInBodyWithRGB) == -1)
            {
                throw std::runtime_error("Couldn't save filtered point cloud to PCD file.");
            }
            std::cout << "Filtered point cloud saved to: " << filteredPointCloudPath << ", the point size is " << scanInBodyWithRGB->size() << std::endl;

            // Transform colored scan into world frame, and combine them into one big colored cloud
            pcl::transformPointCloud(*scanInBodyWithRGB, *scanInWorldWithRGB, transformation_c2w_optimized);
            *cloudInWorldWithRGB += *scanInWorldWithRGB;
        }
    }
}
