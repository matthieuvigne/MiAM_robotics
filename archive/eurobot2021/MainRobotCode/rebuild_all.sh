#!/usr/bin/env bash

# Create tree architecture
cd ~/dev/build/MiAM_robotics
mkdir -p eurobot2021/MainRobotCode/simulation
mkdir -p eurobot2021/MainRobotCode/embedded

# Recompile miam_utils
cd ~/dev/build/MiAM_robotics/miam_utils
cmake ~/dev/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=~/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j4 install
cmake ~/dev/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=~/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j4 install

# Compile for the robot
cd ~/dev/build/MiAM_robotics/eurobot2021/MainRobotCode/embedded
cmake ~/dev/src/MiAM_robotics/eurobot2021/MainRobotCode/embedded -DCMAKE_BUILD_TYPE=Release
make -j4

# Compile for simulation
cd ~/dev/build/MiAM_robotics/eurobot2021/MainRobotCode/simulation
cmake ~/dev/src/MiAM_robotics/eurobot2021/MainRobotCode/simulation -DCMAKE_BUILD_TYPE=Release
make -j4
