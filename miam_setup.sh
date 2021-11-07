#!/usr/bin/env bash
export USERHOME=$(logname)

# Create tree architecture
mkdir -p /home/$USERHOME/dev/build/MiAM_robotics
mkdir -p /home/$USERHOME/dev/install/
mkdir -p /home/$USERHOME/dev/src/

# Install rplidar dependancy
cd /home/$USERHOME/dev/src/
git clone https://github.com/matthieuvigne/rplidar_sdk
mkdir -p /home/$USERHOME/dev/build/rplidar
cd /home/$USERHOME/dev/build/rplidar
cmake /home/$USERHOME/dev/src/rplidar_sdk -DCMAKE_INSTALL_PREFIX=/home/$USERHOME/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j4 install
cmake /home/$USERHOME/dev/src/rplidar_sdk -DCMAKE_INSTALL_PREFIX=/home/$USERHOME/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j4 install

# Add path to bashrc
echo 'export PATH=/home/$USERHOME/dev/install/bin:$PATH' >> /home/$USERHOME/.bashrc
echo 'export PKG_CONFIG_PATH=/home/$USERHOME/dev/install/lib/pkgconfig:$PKG_CONFIG_PATH' >> /home/$USERHOME/.bashrc
echo 'export LD_LIBRARY_PATH=/home/$USERHOME/dev/install/lib:$LD_LIBRARY_PATH' >> /home/$USERHOME/.bashrc

export PATH=/home/$USERHOME/dev/install/bin:$PATH
export PKG_CONFIG_PATH=/home/$USERHOME/dev/install/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/home/$USERHOME/dev/install/lib:$LD_LIBRARY_PATH

echo $PKG_CONFIG_PATH

# Clone git repo
cd /home/$USERHOME/dev/src/
git clone https://github.com/matthieuvigne/MiAM_robotics

# Setup python environment
cd /home/$USERHOME/dev/install
mkdir miam_venv
python3 -m venv miam_venv
echo 'source /home/$USERHOME/dev/install/miam_venv/bin/activate' >> /home/$USERHOME/.bashrc
source /home/$USERHOME/dev/install/miam_venv/bin/activate

cd /home/$USERHOME/dev/src/MiAM_robotics/miam_py
python setup.py install

# Compile and install miam_utils
cd /home/$USERHOME/dev/build/MiAM_robotics
mkdir miam_utils
cd miam_utils

cmake /home/$USERHOME/dev/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=/home/$USERHOME/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j4 install
cmake /home/$USERHOME/dev/src/MiAM_robotics/miam_utils -DCMAKE_INSTALL_PREFIX=/home/$USERHOME/dev/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j4 install

# Compile strategy viewer
cd /home/$USERHOME/dev/build/MiAM_robotics
mkdir -p eurobot2019/StrategyViewer
cd eurobot2019/StrategyViewer
cmake /home/$USERHOME/dev/src/MiAM_robotics/eurobot2019/StrategyViewer -DCMAKE_INSTALL_PREFIX=/home/$USERHOME/dev/install/ -DCMAKE_BUILD_TYPE=Release
make -j4

# Compile robot code
cd /home/$USERHOME/dev/build/MiAM_robotics
mkdir -p eurobot2019/MainRobotCode/
cd eurobot2019/MainRobotCode/
cmake /home/$USERHOME/dev/src/MiAM_robotics/eurobot2019/MainRobotCode -DCMAKE_BUILD_TYPE=Release
make -j4


source /home/$USERHOME/.bashrc
