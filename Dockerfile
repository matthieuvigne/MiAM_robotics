FROM ubuntu:20.04
# Configure base system
RUN apt-get update &&\
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata &&\
    apt-get install -y build-essential pkg-config cmake cmake-curses-gui g++ g++-arm-linux-gnueabihf \
                       python3 python3-venv libgtkmm-3.0-dev python3-tk \
                       libeigen3-dev \
                       gdb ssh sshfs nano wget unzip git &&\
    mkdir /miam_workspace &&\
    echo 'export PATH=/miam_workspace/install/bin:$PATH' >> ~/.bashrc &&\
    echo 'export PKG_CONFIG_PATH=/miam_workspace/install/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc &&\
    echo 'export LD_LIBRARY_PATH=/miam_workspace/install/lib:$LD_LIBRARY_PATH' >> ~/.bashrc &&\
    echo 'source /miam_workspace/install/miam_venv/bin/activate' >> ~/.bashrc

# Add cross-compiled gtkmm-3 library
RUN dpkg --add-architecture armhf &&\
    sed -i "s/deb h/deb [arch=amd64] h/g" /etc/apt/sources.list &&\
    echo "# armhf repositories" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal main restricted" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal-updates main restricted" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal universe" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal-updates universe" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal multiverse" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal-updates multiverse" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal-backports main restricted universe multiverse" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal-security main restricted" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal-security universe" >> /etc/apt/sources.list &&\
    echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports focal-security multiverse" >> /etc/apt/sources.list &&\
    apt-get update &&\
    apt-get install -y libgtkmm-3.0-dev:armhf &&\
    apt-get clean -y &&\
    rm -rf /var/lib/apt/lists

# Cross-compile opencv
RUN cd && wget -O opencv.zip https://github.com/opencv/opencv/archive/4.6.0.zip &&\
    unzip opencv.zip &&\
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.6.0.zip &&\
    unzip opencv_contrib.zip &&\
    rm *zip &&\
    mv opencv_contrib-4.6.0/modules/aruco opencv-4.6.0/modules &&\
    mkdir -p build/opencv && cd build/opencv &&\
    cmake  ../../opencv-4.6.0 -DBUILD_opencv_python_tests=OFF -DBUILD_opencv_python_bindings_generator=OFF -D-DBUILD_opencv_python3=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DCMAKE_TOOLCHAIN_FILE=../opencv-4.6.0/platforms/linux/arm-gnueabi.toolchain.cmake -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=OFF &&\
    make -j8 install

# Cross-compile raspicam
# This step is quite manual: we need to get the shared libs from raspberry pi's firmware,
# and hack the cmake files...
RUN cd &&\
    wget -O libmmal.zip https://github.com/raspberrypi/firmware/archive/refs/tags/1.20181112.zip &&\
    unzip libmmal.zip &&\
    mkdir -p /opt/vc/lib/ &&\
    cp firmware-1.20181112/opt/vc/lib/*mmal* firmware-1.20181112/opt/vc/lib/*vcos* /opt/vc/lib/ &&\
    rm -r libmmal.zip firmware* &&\
    git clone https://github.com/cedricve/raspicam.git && cd raspicam && git checkout 651c56418a5a594fc12f1414eb14f2b899729cb1 &&\
    sed -i '8,10d' src/CMakeLists.txt &&\
    sed -i '106d; 109,129d' CMakeLists.txt &&\
    cd && mkdir -p build/raspicam && cd build/raspicam &&\
    cmake  ../../raspicam  -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_UTILS=OFF -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ -DBUILD_SHARED_LIBS=OFF -DUSE_MMX=OFF -DUSE_O3=OFF -DUSE_FAST_MATH=ON -DUSE_O2=OFF -DUSE_SSE=OFF -DUSE_SSE2=OFF -DUSE_SSE3=OFF &&\
    make -j8 install

# Configure ssh key sharing with host, and keep history between sessions.
RUN mkdir -p /root/.ssh && echo "cp /root/ssh_source/* /root/.ssh/" >> /root/.bashrc &&\
    echo "export PROMPT_COMMAND='history -a' && export HISTFILE=/root/commandhistory/.bash_history" >> /root/.bashrc

# Install other dependencies: HDF5
RUN apt update &&\
    apt install -y libhdf5-dev:armhf &&\
    apt install -y libhdf5-dev &&\
    # Both devel version are in conflict, so we manualy copy the files requires for compilation after the normal install had overwritten the crossed-compiled one.
    cd /tmp/ && apt download libhdf5-dev:armhf &&\
    dpkg -x libhdf5-dev_1.10.4+repack-11ubuntu1_armhf.deb /tmp/ &&\
    cp -r /tmp/usr/lib/arm-linux-gnueabihf/* /usr/lib/arm-linux-gnueabihf/ &&\
    apt-get clean -y
WORKDIR /miam_workspace
