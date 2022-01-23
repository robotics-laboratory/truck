#!/bin/bash

DEBIAN_FRONTEND=noninteractive

# upgrade pkgs

cp ./misc/apt_preferences /etc/apt/preferences

apt update -q && apt upgrade -yq
apt install -qy -yq --no-install-recommends \
   cmake \
   gnupg2 \
   git \
   docker \
   make \
   wget \
   curl \
   python3 \
   python3-dev \
   python3-setuptools \
   python3-pip \
   tar \
   vim

# install jtop

apt install git cmake
apt install -qy -yq --no-install-recommends \
   libhdf5-serial-dev \
   hdf5-tools \
   libatlas-base-dev \
   gfortran

python3 -m pip install -U jetson-stats

# configure docker

usermod -aG docker ${USER}

# setup jetpack

apt install -yq --no-install-recommends nvidia-jetpack

# realsense

LIBRS_VERSION="2.50.0"

cd /tmp
wget -qO - https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBRS_VERSION}.tar.gz | tar -xz\
cd librealsense-${LIBRS_VERSION}
bash scripts/patch-realsense-ubuntu-L4T.sh
bash scripts/setup_udev_rules.sh
rm -rf /tmp/*

# clean
apt autoremove

echo "Please, restart your Jetson!"