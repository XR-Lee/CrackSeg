# Docker Notes

The project was developed in a ROS Noetic-style Linux environment. Docker is useful when you need a reproducible environment for PCL, ROS, Ceres, Qhull, and the manual visualization GUI.

This release does not include a fully pinned Docker image because Iridescence, Qhull, and Ceres installation choices differ across machines. Build or install Ceres 2.1+ before compiling the processor; Ubuntu 20.04's default `libceres-dev` is too old for the current source. The recommended base image is:

```bash
docker pull osrf/ros:noetic-desktop-full
```

Run with X11 forwarding when the manual initial-guess GUI is required:

```bash
xhost +local:docker

docker run -it \
  --workdir=/workspace \
  -v "$PWD":/workspace \
  -e HOME=/root \
  -e QT_X11_NO_MITSHM=1 \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  --name pcd-colorization \
  osrf/ros:noetic-desktop-full \
  bash
```

Inside the container, install the dependencies listed in the repository README, then build:

```bash
cd /workspace/PointCloudProcessor
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```
