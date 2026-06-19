cmake_minimum_required(VERSION 3.16)
project(PointCloudProcessor LANGUAGES C CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
# set(Open3D_DIR ${HOME}/open3d_install/lib/cmake/Open3D)
# message(STATUS "Open3D_DIR: ${Open3D_DIR}")

set(Qhull_DIR "/usr/local/lib/cmake/Qhull")
set(QHULL_LIBRARY /usr/local/lib/libqhull_r.so)
find_library(QHULL_LIBRARIES NAMES qhull_r PATHS /usr/local/lib)


find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Open3D REQUIRED)
find_package(Boost 1.65 REQUIRED COMPONENTS filesystem program_options)
find_package(Ceres 2.1 REQUIRED)
find_package(OpenMP)
# find_package(OpenGL REQUIRED)
find_package(Iridescence REQUIRED)


# 打印OpenCV版本和路径信息
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# 打印OpenCV库详细信息
get_target_property(OpenCV_INCLUDE_DIRS_OPENCV opencv_core INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "OpenCV include directories from target: ${OpenCV_INCLUDE_DIRS_OPENCV}")

get_target_property(OpenCV_LINK_LIBRARIES_OPENCV opencv_core INTERFACE_LINK_LIBRARIES)
message(STATUS "OpenCV link libraries from target: ${OpenCV_LINK_LIBRARIES_OPENCV}")

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

# # 打印GTK信息
# message(STATUS "GTK3 include directories: ${GTK3_INCLUDE_DIRS}")
# message(STATUS "GTK3 libraries: ${GTK3_LIBRARIES}")

if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  eigen_conversions
  cv_bridge
)

# Find the Qhull package
# find_package(Qhull REQUIRED)

# Include Qhull headers
include_directories(${QHULL_INCLUDE_DIRS})

# Global Include directories
include_directories(
  include
  thirdparty/Sophus
  thirdparty/json/include
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# list(APPEND Open3D_LIBRARIES dl)
# if (Open3D_FOUND)
#     message(STATUS "Found Open3D ${Open3D_VERSION}")
#     link_directories(${Open3D_LIBRARY_DIRS})
# endif()

### For VLCAL
add_library(direct_visual_lidar_calibration SHARED
  src/camera/create_camera.cpp
  src/vlcal/common/estimate_fov.cpp
  src/vlcal/common/estimate_pose.cpp
  src/vlcal/common/frame_cpu.cpp
  src/vlcal/common/frame.cpp
  src/vlcal/common/points_color_updater.cpp
  src/vlcal/common/visual_lidar_data.cpp
  src/vlcal/common/visual_lidar_visualizer.cpp
  src/vlcal/calib/view_culling.cpp
  src/vlcal/calib/visual_camera_calibration.cpp
  # src/initial_guess_manual.cpp
)
target_include_directories(direct_visual_lidar_calibration PUBLIC
  include
  thirdparty/Sophus
  thirdparty/json/include
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
  # ${GTK3_INCLUDE_DIRS}
)
target_link_libraries(direct_visual_lidar_calibration
  fmt
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  # ${GTSAM_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Iridescence_LIBRARIES}
  ${QHULL_LIBRARIES}
  # ${GTK3_LIBRARIES}
)

# include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

# include_directories(${Boost_INCLUDE_DIRS})

# add_definitions(${PCL_DEFINITIONS})

# add_executable(${PROJECT_NAME} src/main.cpp src/FrameData.cpp src/PointCloudProcessor.cpp src/RGBFrames.cpp src/cloudSmooth.cpp)

# target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES} Open3D::Open3D)
# # target_link_libraries(${PROJECT_NAME} PRIVATE Open3D::Open3D)

add_executable(${PROJECT_NAME} 
  src/main.cpp 
  src/FrameData.cpp 
  src/PointCloudProcessor.cpp 
  src/cloudSmooth.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${catkin_INCLUDE_DIRS}
  # ${GTK3_INCLUDE_DIRS}

)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  # ${GTK3_LIBRARIES}
  # Open3D::Open3D 
  direct_visual_lidar_calibration
)
