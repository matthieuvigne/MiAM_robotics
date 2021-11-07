#!/usr/bin/env bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

# Install dependancies
apt update
apt install -y git build-essential pkg-config cmake cmake-curses-gui g++ g++-arm-linux-gnueabihf python3 python3-venv libgtkmm-3.0-dev
