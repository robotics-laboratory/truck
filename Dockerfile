ARG BASE_IMAGE=ubuntu:18.04
FROM ${BASE_IMAGE}

ENV ROS_DISTRO=eloquent
ENV GAZEBO_DISTRO=gazebo9

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

ENV SHELL /bin/bash

SHELL ["/bin/bash", "-c"] 
WORKDIR /tmp

# install base packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        cmake \
        less \
        locales \
        build-essential \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        ca-certificates \
        libpython3-dev \
        python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# add repos
RUN echo "Add ros2 repo..." && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    echo "Add gazebo repo..." && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add -

# install ROS packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-launch-xml \
        ros-${ROS_DISTRO}-launch-yaml \
        ros-${ROS_DISTRO}-launch-testing \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-camera-calibration-parsers \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-v4l2-camera \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-image-tools \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-gazebo-ros \
        ros-${ROS_DISTRO}-gazebo-msgs \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-gazebo-plugins \
        python3-colcon-common-extensions \
        python3-rosdep \
        lib${GAZEBO_DISTRO}-dev \
        ${GAZEBO_DISTRO} \
        ${GAZEBO_DISTRO}-common \
        ${GAZEBO_DISTRO}-plugin-base \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# init/update rosdep
RUN apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ros_setup.sh /ros_setup.sh
RUN echo 'source ${ROS_ROOT}/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/ros_setup.sh"]
CMD ["bash"]
WORKDIR /