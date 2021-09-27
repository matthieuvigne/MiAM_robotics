#!/usr/bin/env bash
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

# Create tree architecture
mkdir -p $HOME/dev/build/MiAM_robotics
mkdir -p $HOME/dev/install/
mkdir -p $HOME/dev/src/

# Install dependancies
apt update
apt install -y git build-essential pkg-config cmake cmake-curses-gui g++ g++-arm-linux-gnueabihf python3 python3-venv libgtkmm-3.0-dev

# Add path to bashrc
echo 'export PATH=$HOME/dev/install/bin:$PATH' >> ~/.bashrc
echo 'export PKG_CONFIG_PATH=$HOME/dev/install/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$HOME/dev/install/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

source ~/.bashrc
# Install rplidar dependancy

cd $HOME/dev/src/
git clone https://github.com/matthieuvigne/rplidar_sdk
mkdir -p $HOME/dev/build/rplidar
cd $HOME/dev/build/rplidar
cmake $HOME/dev/src/rplidar_sdk -DCMAKE_INSTALL_PREFIX=$HOME/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j4 install
cmake $HOME/dev/src/rplidar_sdk -DCMAKE_INSTALL_PREFIX=$HOME/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j4 install

# Clone git repo
cd $HOME/dev/src/
git clone https://github.com/matthieuvigne/MiAM_robotics

# Setup python environment
cd $HOME/dev/install
mkdir miam_venv
python3 -m venv miam_venv
echo 'source $HOME/dev/install/miam_venv/bin/activate' >> ~/.bashrc
source ~/.bashrc
cd $HOME/dev/src/MiAM_robotics/miam_py
python setup.py install

# Compile and install miam_utils
cd $HOME/dev/build/MiAM_robotics
mkdir miam_utils
cd miam_utils

cmake $HOME/dev/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=$HOME/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j4 install
cmake $HOME/dev/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=$HOME/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j4 install

# Compile strategy viewer
cd $HOME/dev/build/MiAM_robotics
mkdir -p eurobot2019/StrategyViewer
cd eurobot2019/StrategyViewer
cmake $HOME/dev/src/MiAM_robotics/eurobot2019/StrategyViewer -DCMAKE_INSTALL_PREFIX=$HOME/dev/install/ -DCMAKE_BUILD_TYPE=Release
make -j4

# Compile robot code
cd $HOME/dev/build/MiAM_robotics
mkdir -p eurobot2019/MainRobotCode/
cd eurobot2019/MainRobotCode/
cmake $HOME/dev/src/MiAM_robotics/eurobot2019/MainRobotCode -DCMAKE_BUILD_TYPE=Release
make -j4
