FROM ubuntu:22.04
RUN apt-get update &&\
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata &&\
    apt-get install -y build-essential pkg-config cmake cmake-curses-gui g++ g++-arm-linux-gnueabihf \
                       python3 python3-venv libgtkmm-3.0-dev python3-tk \
                       libeigen3-dev &&\
    apt-get clean -y &&\
    rm -rf /var/lib/apt/lists &&\
    mkdir /miam_workspace &&\
    echo 'export PATH=/miam_workspace/install/bin:$PATH' >> ~/.bashrc &&\
    echo 'export PKG_CONFIG_PATH=/miam_workspace/install/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc &&\
    echo 'export LD_LIBRARY_PATH=/miam_workspace/install/lib:$LD_LIBRARY_PATH' >> ~/.bashrc &&\
    echo 'source /miam_workspace/install/miam_venv/bin/activate' >> ~/.bashrc

WORKDIR /miam_workspace