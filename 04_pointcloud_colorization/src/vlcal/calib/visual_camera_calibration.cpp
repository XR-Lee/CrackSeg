#include <vlcal/calib/visual_camera_calibration.hpp>

#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/autodiff_first_order_function.h>

#include <sophus/se3.hpp>
#include <sophus/ceres_manifold.hpp>

#include <vlcal/costs/nid_cost.hpp>
#include <vlcal/calib/view_culling.hpp>
// #include <vlcal/calib/cost_calculator_nid.hpp>

#include "FrameData.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vlcal
{
  // VisualCameraCalibration::VisualCameraCalibration(
  //   const camera::GenericCameraBase::ConstPtr &proj,
  //   // const std::vector<VisualLiDARData::ConstPtr>& dataset,
  //   const FrameData &frame,
  //   const VisualCameraCalibrationParams &params,
  //   const pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud)
  //   : params(params),
  //     proj(proj),
  //     frame(frame),
  //     point_cloud(point_cloud)
  // {
  // }

  VisualCameraCalibration::VisualCameraCalibration(
      const camera::GenericCameraBase::ConstPtr &proj,
      const std::vector<FrameData::Ptr> &keyframes,
      const VisualCameraCalibrationParams &params)
      : proj(proj),
        keyframes(keyframes),
        params(params)
  {
  }

  Eigen::Isometry3d VisualCameraCalibration::calibrate(const Eigen::Isometry3d &init_T_camera_lidar)
  {
    Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar;
    // Outer loop
    for (int i = 0; i < params.max_outer_iterations; i++)
    {
      Eigen::Isometry3d new_T_camera_lidar;
      switch (params.registration_type)
      {
      case RegistrationType::NID_BFGS:
        new_T_camera_lidar = estimate_pose_bfgs(T_camera_lidar);
        break;

        // case RegistrationType::NID_NELDER_MEAD:
        //   new_T_camera_lidar = estimate_pose_nelder_mead(T_camera_lidar);
        //   break;
      }

      const Eigen::Isometry3d delta = new_T_camera_lidar.inverse() * T_camera_lidar;
      T_camera_lidar = new_T_camera_lidar;

      const double delta_t = delta.translation().norm();
      const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();
      const bool converged = delta_t < params.delta_trans_thresh && delta_r < params.delta_rot_thresh;

      std::cout << boost::format("\n delta_t: %.4f [m] delta_r: %.6f [rad]") % delta_t % delta_r << std::endl;
      std::cout << (converged ? "Outer loop converged" : "Outer loop not converged, Re-run inner optimization with the new viewpoint") << std::endl;

      if (converged)
      {
        break;
      }
    }

    return T_camera_lidar;
  }

  struct MultiNIDCost
  {
  public:
    MultiNIDCost(const Sophus::SE3d &init_T_camera_lidar) : init_T_camera_lidar(init_T_camera_lidar) {}

    void add(const std::shared_ptr<NIDCost> &cost) { costs.emplace_back(cost); }

    template <typename T>
    bool operator()(const T *params, T *residual) const
    {
      std::vector<double> values(Sophus::SE3d::num_parameters);
      std::transform(params, params + Sophus::SE3d::num_parameters, values.begin(), [](const auto &x)
                     { return get_real(x); });
      const Eigen::Map<const Sophus::SE3d> T_camera_lidar(values.data());
      const Sophus::SE3d delta = init_T_camera_lidar.inverse() * T_camera_lidar;

      if (delta.translation().norm() > 0.2 || Eigen::AngleAxisd(delta.rotationMatrix()).angle() > 2.0 * M_PI / 180.0)
      {
        return false;
      }

      std::vector<bool> results(costs.size());
      std::vector<T> residuals(costs.size());

#pragma omp parallel for
      for (int i = 0; i < costs.size(); i++)
      {
        results[i] = (*costs[i])(params, &residuals[i]);
      }

      for (int i = 1; i < costs.size(); i++)
      {
        residuals[0] += residuals[i];
      }

      *residual = residuals[0];

      return std::count(results.begin(), results.end(), false) == 0;
    }

  private:
    Sophus::SE3d init_T_camera_lidar;
    std::vector<std::shared_ptr<NIDCost>> costs;
  };

  struct IterationCallbackWrapper : public ceres::IterationCallback
  {
  public:
    IterationCallbackWrapper(const std::function<ceres::CallbackReturnType(const ceres::IterationSummary &)> &callback) : callback(callback) {}

    virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) { return callback(summary); }

  private:
    std::function<ceres::CallbackReturnType(const ceres::IterationSummary &)> callback;
  };

  Eigen::Isometry3d VisualCameraCalibration::estimate_pose_bfgs(const Eigen::Isometry3d &init_T_camera_lidar)
  {
    // Hidden point removal
    ViewCullingParams view_culling_params;
    view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
    // ViewCulling view_culling(proj, {dataset.front()->image.cols, dataset.front()->image.rows}, view_culling_params);
    // std::cout << "before view_culling!" << std::endl;
    ViewCulling view_culling(proj, {4096, 3000}, view_culling_params); // TODO: hardcode

    Sophus::SE3d T_camera_lidar(init_T_camera_lidar.matrix()); // Init extrinsics

    std::vector<std::shared_ptr<NIDCost>> nid_costs; // create nid costs for each frame

    // for each frames, first remove hidden points and then create NID cost
    for (auto keyframe : keyframes)
    {
      // First read out the point cloud from the keyframe->culledPCDPath
      // Remove hidden points
      pcl::PointCloud<pcl::PointXYZI>::Ptr culledPCD(new pcl::PointCloud<pcl::PointXYZI>());

      if (pcl::io::loadPCDFile<pcl::PointXYZI>(keyframe->culledPCDPath, *culledPCD) == -1)
      {
        std::cerr << "Couldn't read file " << keyframe->culledPCDPath << std::endl;
        continue;
      }
      std::cout << "Success read file " << keyframe->culledPCDPath << std::endl;

      // create normalized image in CV_64FC1 format from original image for NID cost
      cv::Mat normalized_image;
      cv::Mat orig_image = cv::imread(keyframe->imagePath);
      std::cout << "Success read image file: " << keyframe->imagePath << "\n" << std::endl;
      orig_image.convertTo(normalized_image, CV_64FC1, 1.0 / 255.0);

      // create NIDCost object with the normalized image, culled points and projection matrix
      std::shared_ptr<NIDCost> nid_cost(new NIDCost(proj, normalized_image, culledPCD, params.nid_bins));
      // add the NIDCost object to the vector
      nid_costs.emplace_back(nid_cost);
    }

    // // If only one pcd is available
    // pcl::PointCloud<pcl::PointXYZI>::Ptr culled_points = view_culling.cull(point_cloud, init_T_camera_lidar);
    // std::cout << "after view_culling!" << std::endl;

    // // create normalized image in CV_64FC1 format from original image for NID cost
    // cv::Mat normalized_image;
    // // dataset[i]->image.convertTo(normalized_image, CV_64FC1, 1.0 / 255.0); // TODO: replace with raw image frame
    // cv::Mat orig_image = cv::imread(frame.imagePath);

    // orig_image.convertTo(normalized_image, CV_64FC1, 1.0 / 255.0);

    // // create NIDCost object with the normalized image, culled points and projection matrix
    // std::cout << "Init nid_cost" << std::endl;
    // std::shared_ptr<NIDCost> nid_cost(new NIDCost(proj, normalized_image, culled_points, params.nid_bins));
    // // add the NIDCost object to the vector
    // nid_costs.emplace_back(nid_cost);

    auto sum_nid = new MultiNIDCost(T_camera_lidar);
    for (const auto &nid_cost : nid_costs)
    {
      sum_nid->add(nid_cost);
    }

    auto cost = new ceres::AutoDiffFirstOrderFunction<MultiNIDCost, Sophus::SE3d::num_parameters>(sum_nid);
    ceres::GradientProblem problem(cost, new Sophus::Manifold<Sophus::SE3>());
    // ceres::GradientProblem problem(cost);

    ceres::GradientProblemSolver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.update_state_every_iteration = true;
    options.line_search_direction_type = ceres::BFGS;

std::cout << "Before adding callback!" << std::endl;

    // options.callbacks.emplace_back(new IterationCallbackWrapper([&](const ceres::IterationSummary &summary)
    //                                                             {
    // params.callback(Eigen::Isometry3d(T_camera_lidar.matrix()));
    // return ceres::CallbackReturnType::SOLVER_CONTINUE; }));

    options.callbacks.emplace_back(new IterationCallbackWrapper([&](const ceres::IterationSummary &summary) {
        std::cout << "Inside callback: Iteration " << summary.iteration << std::endl;
        // params.callback(Eigen::Isometry3d(T_camera_lidar.matrix()));
        if (params.callback) {
            params.callback(Eigen::Isometry3d(T_camera_lidar.matrix()));
        }
        std::cout << "Callback done: Iteration " << summary.iteration << std::endl;
        return ceres::CallbackReturnType::SOLVER_CONTINUE;
    }));

    std::cout << "Before Solve!" << std::endl;

    ceres::GradientProblemSolver::Summary summary;
    ceres::Solve(options, problem, T_camera_lidar.data(), &summary);

    // options.callbacks.emplace_back(new IterationCallbackWrapper([&](const ceres::IterationSummary &summary)
    //                                                             {
    //     if (params.callback) {
    //         params.callback(Eigen::Isometry3d(T_camera_lidar.matrix()));
    //     }
    //     return ceres::CallbackReturnType::SOLVER_CONTINUE; }));

    // ceres::GradientProblemSolver::Summary summary;
    // ceres::Solve(options, problem, T_camera_lidar.data(), &summary);

    std::cout << "ceres solve done!" << std::endl;
    std::cout << boost::format("Inner optimization (BFGS) terminated after %d iterations") % summary.iterations.size() << std::endl;
    std::cout << boost::format("Final cost: %.3f") % summary.final_cost << std::endl;
    std::cout << "--- T_camera_lidar ---" << std::endl
              << T_camera_lidar.matrix();

    return Eigen::Isometry3d(T_camera_lidar.matrix());
  }
}
