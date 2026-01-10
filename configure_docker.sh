#!/usr/bin/env bash

# This script is meant to be run inside docker: check that.

if ! [ -f /.dockerenv ]; then
   echo "This script must run inside the docker container"
   exit -1
fi

# Create directory structure
mkdir -p /miam_workspace/install
mkdir -p /miam_workspace/build/cross_compile
mkdir -p /miam_workspace/build/native_compile

# Build code
cd /miam_workspace/build/cross_compile
cmake /miam_workspace/src/MiAM_robotics -DCMAKE_INSTALL_PREFIX=/miam_workspace/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=ON
make -j8 install

cd /miam_workspace/build/native_compile
cmake /miam_workspace/src/MiAM_robotics -DCMAKE_INSTALL_PREFIX=/miam_workspace/install/ -DCMAKE_BUILD_TYPE=Release -DCROSS_COMPILE=OFF
make -j8 install


# Setup python environment
cd /miam_workspace/install
mkdir -p miam_venv
python3 -m venv miam_venv
source /miam_workspace/install/miam_venv/bin/activate

cd /miam_workspace/src/MiAM_robotics/miam_py
pip install -U pip
pip install -e .

