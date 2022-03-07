#!/usr/bin/env bash
export USERHOME=$(logname)

# Create tree architecture
mkdir -p ~/dev/build/MiAM_robotics
mkdir -p ~/dev/install/
mkdir -p ~/dev/src/

# Install rplidar dependancy
cd ~/dev/src/
git clone https://github.com/matthieuvigne/rplidar_sdk
mkdir -p ~/dev/build/rplidar
cd ~/dev/build/rplidar
cmake ~/dev/src/rplidar_sdk -DCMAKE_INSTALL_PREFIX=~/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j4 install
cmake ~/dev/src/rplidar_sdk -DCMAKE_INSTALL_PREFIX=~/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j4 install

# Add path to bashrc
echo 'export PATH=~/dev/install/bin:$PATH' >> ~/.bashrc
echo 'export PKG_CONFIG_PATH=~/dev/install/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=~/dev/install/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

export PATH=~/dev/install/bin:$PATH
export PKG_CONFIG_PATH=~/dev/install/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=~/dev/install/lib:$LD_LIBRARY_PATH

echo $PKG_CONFIG_PATH

# Clone git repo
cd ~/dev/src/
git clone https://github.com/matthieuvigne/MiAM_robotics

# Setup python environment
cd ~/dev/install
mkdir miam_venv
python3 -m venv miam_venv
echo 'source ~/dev/install/miam_venv/bin/activate' >> ~/.bashrc
source ~/dev/install/miam_venv/bin/activate

cd ~/dev/src/MiAM_robotics/miam_py
python setup.py install

# Compile and install miam_utils
cd ~/dev/build/MiAM_robotics
mkdir miam_utils
cd miam_utils

cmake ~/dev/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=~/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j4 install
cmake ~/dev/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=~/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j4 install

# Compile strategy viewer
cd ~/dev/build/MiAM_robotics
mkdir -p eurobot2019/StrategyViewer
cd eurobot2019/StrategyViewer
cmake ~/dev/src/MiAM_robotics/eurobot2019/StrategyViewer -DCMAKE_INSTALL_PREFIX=~/dev/install/ -DCMAKE_BUILD_TYPE=Release
make -j4

# Compile robot code
cd ~/dev/build/MiAM_robotics
mkdir -p eurobot2019/MainRobotCode/
cd eurobot2019/MainRobotCode/
cmake ~/dev/src/MiAM_robotics/eurobot2019/MainRobotCode -DCMAKE_BUILD_TYPE=Release
make -j4


source ~/.bashrc
