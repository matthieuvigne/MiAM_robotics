#!/usr/bin/env bash

# This script is meant to be run inside docker: check that.

if ! [ -f /.dockerenv ]; then
   echo "This script must run inside the docker container"
   exit -1
fi

# Create directory structure
cd /miam_workspace

mkdir -p build/rplidar
mkdir -p build/miam_utils
mkdir -p build/vision
mkdir -p build/eurobot2025/embedded
mkdir -p build/eurobot2025/simulation
mkdir -p build/eurobot2025/simulation/logs
mkdir install

# Compile and install rplidar
cd build/rplidar

cmake /miam_workspace/src/MiAM_robotics/rplidar_sdk -DCMAKE_INSTALL_PREFIX=/miam_workspace/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j8 install

cmake /miam_workspace/src/MiAM_robotics/rplidar_sdk -DCMAKE_INSTALL_PREFIX=/miam_workspace/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j8 install

# Compile and install miam_utils
cd /miam_workspace/build/miam_utils

cmake /miam_workspace/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=/miam_workspace/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j8 install

cmake /miam_workspace/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=/miam_workspace/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j8 install

# Compile and install miam_vision_arm
cd /miam_workspace/build/vision

cmake /miam_workspace/src/MiAM_robotics/vision -DCMAKE_INSTALL_PREFIX=/miam_workspace/install/ -DCMAKE_BUILD_TYPE=Release
make -j8 install

# Setup python environment
cd /miam_workspace/install
mkdir -p miam_venv
python3 -m venv miam_venv
source /miam_workspace/install/miam_venv/bin/activate

cd /miam_workspace/src/MiAM_robotics/miam_py
pip install -U pip
pip install -e .

# Compile vision code
cd /miam_workspace/build/vision
cmake /miam_workspace/src/MiAM_robotics/vision -DCMAKE_BUILD_TYPE=Release
make -j8

# Compile robot code
cd /miam_workspace/build/eurobot2025/embedded
cmake /miam_workspace/src/MiAM_robotics/eurobot2025/MainRobotCode/embedded/ -DCMAKE_BUILD_TYPE=Release
make -j8

# Compile simulation code
cd /miam_workspace/build/eurobot2025/simulation
cmake /miam_workspace/src/MiAM_robotics/eurobot2025/MainRobotCode/simulation/ -DCMAKE_BUILD_TYPE=Release
make -j8

