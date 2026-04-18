#!/usr/bin/env bash
set -euo pipefail

echo "[manual-loop-closure] Installing Ubuntu 20.04 / ROS Noetic dependencies..."
echo "[manual-loop-closure] This script assumes your ROS Noetic apt source is already configured."

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  pkg-config \
  python3-pip \
  python3-venv \
  python3-catkin-tools \
  libpcl-dev \
  libopencv-dev \
  libboost-all-dev \
  libgeographic-dev \
  libzstd-dev \
  ros-noetic-roscpp \
  ros-noetic-rospy \
  ros-noetic-rosbag \
  ros-noetic-tf \
  ros-noetic-geometry-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-std-msgs \
  ros-noetic-image-transport \
  ros-noetic-cv-bridge \
  ros-noetic-pcl-conversions \
  ros-noetic-visualization-msgs \
  ros-noetic-std-srvs

echo
echo "[manual-loop-closure] Done."
echo "[manual-loop-closure] Next steps:"
echo "  1) Install GTSAM 4.2+ if it is not already available to CMake."
echo "  2) Create the Python GUI environment:"
echo "       conda env create -f environment.yml"
echo "       conda activate manual-loop-closure"
echo "  3) Build the backend:"
echo "       bash scripts/build_backend_catkin.sh"
