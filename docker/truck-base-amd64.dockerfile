FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=galactic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

WORKDIR /tmp

### INSTALL CMAKE

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        software-properties-common \
        apt-transport-https \
        ca-certificates \
        gnupg \
        wget \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add - \
    && apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' \
    && apt-get update -q \
    && apt-get install -yq --no-install-recommends cmake \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

### INSTALL OPENCV

ARG OPENCV_VERSION="4.5.0"

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        build-essential \
        gfortran \
        make \
        git \
        file \
        libatlas-base-dev \
        libavcodec-dev \
        libavformat-dev \
        libavresample-dev \
        libcanberra-gtk3-module \
        libdc1394-22-dev \
        libeigen3-dev \
        libglew-dev \
        libglu1-mesa-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-good1.0-dev \
        libgstreamer1.0-dev \
        libgtk-3-dev \
        libjpeg-dev \
        libjpeg8-dev \
        libjpeg-turbo8-dev \
        liblapack-dev \
        liblapacke-dev \
        libopenblas-dev \
        libpng-dev \
        libpostproc-dev \
        libswscale-dev \
        libtbb-dev \
        libtbb2 \
        libtesseract-dev \
        libtiff-dev \
        libv4l-dev \
        libxine2-dev \
        libxvidcore-dev \
        libx264-dev \
        libgtkglext1 \
        libgtkglext1-dev\
        mesa-common-dev \
        pkg-config \
        python3-pip \
        python3-dev \
        python3-numpy \
        python3-distutils \
        python3-setuptools \
        qv4l2 \
        tar \
        v4l-utils \
        v4l2ucp \
        zlib1g-dev \
        wget \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN wget -qO - https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && wget -qO - https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && cd opencv-${OPENCV_VERSION} && mkdir -p build && cd build \
    && OPENCV_MODULES=(core calib3d features2d flann highgui imgcodecs photo \
        stitching video videoio aruco bgsegm ccalib optflow rgbd sfm stereo \
        surface_matching xfeatures2d ximgproc xphoto) \
    && cmake .. \
        -DBUILD_LIST=$(echo ${OPENCV_MODULES[*]} | tr ' '  ',') \
        -DCMAKE_BUILD_TYPE=RELEASE \
        -DWITH_GTK=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_opencv_apps=OFF \
        -DBUILD_opencv_python2=OFF \
        -DBUILD_opencv_python3=ON \
        -DBUILD_opencv_java=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DEIGEN_INCLUDE_PATH=/usr/include/eigen3 \
        -DWITH_EIGEN=ON \
        -DOPENCV_ENABLE_NONFREE=ON \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV_VERSION}/modules \
        -DWITH_GSTREAMER=ON \
        -DWITH_LIBV4L=ON \
        -DWITH_OPENCL=ON \
        -DWITH_IPP=OFF \
        -DWITH_TBB=ON \
        -DBUILD_TIFF=ON \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_TESTS=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### INSTALL REALSENSE2

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        gnupg2 \
        libgtk-3-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libssl-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        lsb-release \
        pkg-config \
        software-properties-common \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -yq --no-install-recommends \
        librealsense2-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

# ### INSTALL RTAB-MAP

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        build-essential \
        git \
        make \
        libeigen3-dev \
        libpython3-dev \
        python3-dev \
        libyaml-cpp-dev \
        libboost-all-dev \
        libtbb-dev \
        libsqlite3-dev \
        libpcl-dev \
        libproj-dev \
        libqt5svg5-dev \
        libsuitesparse-dev \
        tar \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ARG G2O_HASH="b1ba729aa569267e179fa2e237db0b3ad5169e2e"

RUN git clone https://github.com/RainerKuemmerle/g2o.git \
    && cd g2o && git checkout ${G2O_HASH} \
    && mkdir -p build && cd build \
    && cmake .. \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DG2O_BUILD_APPS=OFF \
        -DG2O_BUILD_EXAMPLES=OFF \
        -DG2O_USE_OPENGL=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

ARG GTSAM_VERSION="4.1.1"

RUN wget -qO - https://github.com/borglab/gtsam/archive/refs/tags/${GTSAM_VERSION}.tar.gz | tar -xz \
    && cd gtsam-${GTSAM_VERSION} && mkdir -p build && cd build \
    && cmake .. \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

ARG LIBNABO_VERSION="1.0.7"

RUN wget -qO - https://github.com/ethz-asl/libnabo/archive/refs/tags/${LIBNABO_VERSION}.tar.gz | tar -xz \
    && cd libnabo-${LIBNABO_VERSION} && mkdir -p build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

ARG LIBPOINTMATCHER_VERSION="1.3.1"

RUN wget -qO - https://github.com/ethz-asl/libpointmatcher/archive/refs/tags/${LIBPOINTMATCHER_VERSION}.tar.gz | tar -xz \
    && cd libpointmatcher-${LIBPOINTMATCHER_VERSION} && mkdir -p build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

# Use ROS release repo, version is not fixed!
ARG RTAB_MAP_BRANCH="release/${ROS_DISTRO}/rtabmap"

RUN git clone https://github.com/introlab/rtabmap-release.git \
    && cd rtabmap-release && git checkout ${RTAB_MAP_BRANCH} \
    && mkdir -p build && cd build \
    && cmake .. \
        -DBUILD_APP=OFF \
        -DBUILD_TOOLS=ON \
        -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### INSTALL ROS2

RUN wget -q https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

ENV GAZEBO="gazebo11"

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        apt-utils \
        build-essential \
        ${GAZEBO} \
        ${GAZEBO}-common \
        ${GAZEBO}-plugin-base \
        git \
        imagemagick \
        libjansson-dev \
        make \
        libasio-dev \
        libboost-dev \
        libbullet-dev \
        lib${GAZEBO}-dev \
        libpython3-dev \
        libtinyxml-dev \
        locales \
        python3-bson \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-numpy \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        python3-rosinstall-generator \
        libtinyxml2-dev \
        libcunit1-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN pip3 install --no-cache-dir -U \
        argcomplete \
        importlib-metadata \
        importlib-resources \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest

RUN mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT} \
    && rosinstall_generator \
            --rosdistro ${ROS_DISTRO} \
            --exclude librealsense2 rtabmap libg2o \
            --deps \
        image_geometry \
        image_pipeline \
        image_transport \
        compressed_image_transport \
        compressed_depth_image_transport \
        cv_bridge \
        gazebo_ros_pkgs \
        gazebo_ros2_control \
        launch_xml \
        launch_yaml \
        nav2_common \
        pcl_conversions \
        realsense2_camera \
        realsense2_description \
        ros_base \
        ros2_control \
        ros2_controllers \
        rosbridge_suite \
        rtabmap_ros \
        vision_opencv \
        vision_msgs \
    > ros2.${ROS_DISTRO}.rosinstall \
    && vcs import src < ros2.${ROS_DISTRO}.rosinstall > /dev/null

RUN apt-get update -q \
    && rosdep init \
    && rosdep update \
    && rosdep install -qy --ignore-src  \
        --rosdistro ${ROS_DISTRO} \
        --from-paths ${ROS_ROOT}/src \
        --skip-keys fastcdr \
        --skip-keys gazebo11 \
        --skip-keys libg2o \
        --skip-keys libgazebo11-dev \
        --skip-keys librealsense2 \
        --skip-keys libopencv-dev \
        --skip-keys libopencv-contrib-dev \
        --skip-keys libopencv-imgproc-dev \
        --skip-keys python3-opencv \
        --skip-keys python3-opencv \
        --skip-keys rti-connext-dds-5.3.1 \
        --skip-keys rtabmap \
        --skip-keys urdfdom_headers \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN cd ${ROS_ROOT}/src \
    && colcon build \
        --merge-install \
        --install-base ${ROS_ROOT} \
        --cmake-args -DBUILD_TESTING=OFF \ 
        --catkin-skip-building-tests \
    && echo 'source ${ROS_ROOT}/setup.bash' >> /root/.bashrc \
    && rm -rf /tmp/*

ENV GZWEB_VERSION="1.4.1"
ENV GZWEB_PATH=/opt/gzweb

RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash \
    && source ${HOME}/.nvm/nvm.sh \
    && nvm install 9 \
    && mkdir -p ${GZWEB_PATH} \
    && wget -qO - https://github.com/osrf/gzweb/archive/refs/tags/gzweb_${GZWEB_VERSION}.tar.gz | tar -xz -C ${GZWEB_PATH} --strip-components 1 \
    && cd ${GZWEB_PATH} \
    && source /usr/share/gazebo/setup.sh \
    && npm run deploy --- -m -t

### INSTALL DEV PKGS

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        build-essential \
        gfortran \
        clang-format \
        curl \
        file \
        gfortran \
        git \
        gnupg2 \
        file \
        htop \
        httpie \
        less \
        make \
        nlohmann-json-dev \
        python3 \
        python3-dev \
        python3-distutils \
        python3-pip \
        python3-setuptools \
        tar \
        tmux \
        vim \
        wget \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

### SETUP ENTRYPOINT

COPY /entrypoint.bash /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]
WORKDIR /truck
