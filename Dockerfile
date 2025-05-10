FROM ubuntu:22.04
# Configure base system
RUN apt-get update &&\
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata &&\
    apt-get install -y build-essential pkg-config cmake cmake-curses-gui g++ g++-aarch64-linux-gnu \
                       python3 python3-venv libgtkmm-3.0-dev python3-tk \
                       libeigen3-dev libhdf5-dev \
                       gdb ssh sshfs nano wget unzip git \
                       gcc git gnuplot doxygen graphviz \
                       '^libxcb.*-dev' '^libxkb.*-dev' &&\
    mkdir /miam_workspace &&\
    echo 'export PATH=/miam_workspace/install/bin:$PATH' >> ~/.bashrc &&\
    echo 'export PKG_CONFIG_PATH=/miam_workspace/install/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc &&\
    echo 'export LD_LIBRARY_PATH=/miam_workspace/install/lib:$LD_LIBRARY_PATH' >> ~/.bashrc &&\
    echo 'source /miam_workspace/install/miam_venv/bin/activate' >> ~/.bashrc


# Add cross-compiled gtkmm-3 and opencv library
RUN dpkg --add-architecture arm64 &&\
    sed -i "s/deb h/deb [arch=amd64] h/g" /etc/apt/sources.list &&\
    echo "# arm64 repositories" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy main restricted" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy universe" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-updates universe" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy multiverse" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-updates multiverse" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-security universe" >> /etc/apt/sources.list &&\
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-security multiverse" >> /etc/apt/sources.list &&\
    apt-get update &&\
    apt-get install -y libgtkmm-3.0-dev:arm64 libopencv-dev:arm64 &&\
    apt-get clean -y &&\
    rm -rf /var/lib/apt/lists

# Install acado
RUN  cd / &&\
     git clone https://github.com/acado/acado.git -b stable &&\
     mkdir /acado/build &&\
     cd /acado/build &&\
    cmake /acado &&\
    make -j8 &&\
    echo "source /acado/build/acado_env.sh" >> /root/.bashrc

# Configure ssh key sharing with host, and keep history between sessions.
RUN mkdir -p /root/.ssh && echo "cp /root/ssh_source/* /root/.ssh/" >> /root/.bashrc &&\
    echo "export PROMPT_COMMAND='history -a' && export HISTFILE=/root/commandhistory/.bash_history" >> /root/.bashrc

WORKDIR /miam_workspace
