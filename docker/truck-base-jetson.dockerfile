FROM nvcr.io/nvidia/l4t-base:r32.6.1

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME="/usr/local/cuda"
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

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

# INSTALL OPENCV
ARG OPENCV_VERSION="4.5.0"

RUN apt-get update -yq && \
    apt-get install -yq --no-install-recommends \
        build-essential \
        gfortran \
        make \
        git \
        file \
        tar \
        python3-pip \
        python3-dev \
        python3-numpy \
        python3-distutils \
        python3-setuptools \
        libatlas-base-dev \
        libavcodec-dev \
        libavformat-dev \
        libavresample-dev \
        libcanberra-gtk3-module \
        libdc1394-22-dev \
        libeigen3-dev \
        libglew-dev \
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
        libgtkglext1-dev \
        pkg-config \
        qv4l2 \
        v4l-utils \
        v4l2ucp \
        zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN wget -qO - https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && wget -qO - https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.tar.gz | tar -xz \
    && cd opencv-${OPENCV_VERSION} && mkdir -p build && cd build \
    && OPENCV_MODULES=(core calib3d features2d flann highgui imgcodecs photo python stitching video videoio \
        aruco bgsegm ccalib cudaarithm cudabgsegm cudacodec cudafeatures2d cudafilters cudaimgproc \
        cudaoptflow cudastereo cudawarping cudev optflow rgbd sfm stereo surface_matching \
        xfeatures2d ximgproc xphoto) \
    && cmake .. \
        -DBUILD_LIST=$(echo ${OPENCV_MODULES[*]} | tr ' ' ',') \
        -DCMAKE_BUILD_TYPE=RELEASE \
        -DWITH_GTK=ON \
        -DBUILD_opencv_apps=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_opencv_python2=OFF \
        -DBUILD_opencv_python3=ON \
        -DBUILD_opencv_java=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCUDA_ARCH_BIN=5.3,6.2,7.2 \
        -DCUDA_ARCH_PTX= \
        -DCUDA_FAST_MATH=ON \
        -DCUDNN_INCLUDE_DIR=/usr/include \
        -DEIGEN_INCLUDE_PATH=/usr/include/eigen3 \
        -DWITH_EIGEN=ON \
        -DENABLE_NEON=ON \
        -DOPENCV_DNN_CUDA=ON \
        -DOPENCV_ENABLE_NONFREE=ON \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV_VERSION}/modules \
        -DWITH_CUBLAS=ON \
        -DWITH_CUDA=ON \
        -DWITH_CUDNN=ON \
        -DWITH_GSTREAMER=ON \
        -DWITH_LIBV4L=ON \
        -DWITH_OPENGL=ON \
        -DWITH_OPENCL=OFF \
        -DWITH_IPP=OFF \
        -DWITH_TBB=ON \
        -DBUILD_TIFF=ON \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_TESTS=OFF \
    && make -j$(nproc) install && rm -rf /tmp/*

# INSTALL LIBREALSENSE
ARG LIBRS_VERSION="2.50.0"

RUN apt-get update -yq \
    && apt-get install -yq --no-install-recommends \
        build-essential \
        git \
        make \
        libssl-dev \
        libusb-1.0-0-dev \
        libudev-dev \
        libomp-dev \
        pkg-config \
        sudo \
        tar \
        wget \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN wget -qO - https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBRS_VERSION}.tar.gz | tar -xz \
    && cd librealsense-${LIBRS_VERSION} \
    && bash scripts/setup_udev_rules.sh \
    && mkdir -p build && cd build \
    && cmake .. \
        -DBUILD_EXAMPLES=true \
        -DCMAKE_BUILD_TYPE=release \
        -DFORCE_RSUSB_BACKEND=false \
        -DBUILD_WITH_CUDA=true \
        -DBUILD_WITH_OPENMP=true \
        -DBUILD_PYTHON_BINDINGS=true \
        -DBUILD_WITH_TM2=false \
    && make -j$(($(nproc)-1)) install \
    && rm -rf /tmp/*

# INSTALL PYTORCH
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-dev \
        libopenblas-dev \
        libopenmpi2 \
        openmpi-bin \
        openmpi-common \
        gfortran \
        git \
        wget \
        build-essential \
        libjpeg-dev \
        zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* apt-get clean

ARG PILLOW_VERSION=pillow<7

RUN pip3 install --no-cache-dir \
    setuptools \
    Cython \
    wheel \
    numpy \
    ${PILLOW_VERSION}

ARG PYTORCH_URL=https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl
ARG PYTORCH_WHL=torch-1.10.0-cp36-cp36m-linux_aarch64.whl

RUN wget --no-check-certificate -qO ${PYTORCH_WHL} ${PYTORCH_URL} \
    && pip3 install --no-cache-dir ${PYTORCH_WHL} \
    && rm -rf /tmp/*

ARG TORCHVISION_VERSION=0.11.1

RUN wget -qO - https://github.com/pytorch/vision/archive/refs/tags/v${TORCHVISION_VERSION}.tar.gz | tar -xz \
    && cd vision-${TORCHVISION_VERSION} \
    && python3 setup.py install \
    && rm -rf /tmp/*
 
# INSTALL RTAB-MAP

ARG G2O_VERSION="20201223_git"

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        git \
        make \
        libeigen3-dev \
        libpcl-dev \
        libpython3-dev \
        python3-dev \
        libyaml-cpp-dev \
        libboost-all-dev \
        libtbb-dev \
    && rm -rf /var/lib/apt/lists/* apt-get clean

RUN wget -qO - https://github.com/RainerKuemmerle/g2o/archive/refs/tags/${G2O_VERSION}.tar.gz | tar -xz \
    && cd g2o-${G2O_VERSION} && mkdir -p build && cd build \
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

ARG RTAB_MAP_VERSION="0.20.8"

RUN wget -qO - https://github.com/introlab/rtabmap/archive/refs/tags/${RTAB_MAP_VERSION}.tar.gz | tar -xz \
    && cd rtabmap-${RTAB_MAP_VERSION} && mkdir -p build && cd build \
    && cmake .. \
        -DWITH_PYTHON=ON \
        -DWITH_TORCH=ON \
        -DTorch_DIR=${PYTORCH_PATH}/share/cmake/Torch \
    && make -j$(nproc) install \
    && rm -rf /tmp/*

### INSTALL ROS2

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/* && apt-get clean
ENV GAZEBO_VERSION="gazebo9"

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        apt-utils \
        build-essential \
        curl \
        git \
        make \
        libbullet-dev \
        libpython3-dev \
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
        libasio-dev \
        libtinyxml2-dev \
        libcunit1-dev \
        lib${GAZEBO_VERSION}-dev \
        ${GAZEBO_VERSION} \
        ${GAZEBO_VERSION}-common \
        ${GAZEBO_VERSION}-plugin-base \
        wget \
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

ENV ROS_DISTRO=galactic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

RUN mkdir -p ${ROS_DISTRO}/src && cd ${ROS_DISTRO} \
    && rosinstall_generator --deps --rosdistro ${ROS_DISTRO} \
        ros_base \
        image_geometry \
        image_pipeline \
        image_transport \
        camera_calibration_parsers \
        camera_info_manager \
        compressed_image_transport \
        compressed_depth_image_transport \
        cv_bridge \
        diagnostic_updater \
        launch_xml \
        launch_yaml \
        nav2_common \
        pcl_conversions \
        realsense2_camera \
        realsense2_description \
        rosbridge_suite \
        rtabmap_ros\
        v4l2_camera \
        vision_opencv \
        vision_msgs \
    > ros2.${ROS_DISTRO}.rosinstall \
    && cat ros2.${ROS_DISTRO}.rosinstall \
    && vcs import src < ros2.${ROS_DISTRO}.rosinstall

RUN apt-get update -q \
    && cd ${ROS_DISTRO}/src \
    && rosdep init \
    && rosdep update \
    && SKIP_KEYS=(fastcdr rti-connext-dds-5.3.1 urdfdom_headers librealsense2 \
            libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python3-opencv) \
    && rosdep install -qy --ignore-src --from-paths . \
        --rosdistro ${ROS_DISTRO} \
        --skip-keys "${SKIP_KEYS[*]}" \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN cd ${ROS_DISTRO}/src \
    && colcon build \
        --merge-install \
        --install-base ${ROS_ROOT} \
    && echo 'source ${ROS_ROOT}/setup.bash' >> /root/.bashrc \
    && rm -rf /tmp/*

RUN pip3 install git+https://github.com/colcon/colcon-bazel#egg=colcon-bazel --upgrade

RUN wget -qO /usr/local/bin/bazel https://github.com/bazelbuild/bazel/releases/download/4.2.2/bazel-4.2.2-linux-arm64 \
    && [ "$(sha256sum bazel | awk '{print $1}')" = "de5ddfebe1a769f067c77e29c197fc9a5bc855a502316070f6bb0fbac9ac37f8" ] \
    && chmod +x /usr/local/bin/bazel

RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y \
    && apt-get update \
    && apt-get install gcc-9 g++-9 \
    && update-alternatives --install /usr/bin/cc cc /usr/bin/gcc-9 50 \
    && update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++-9 50 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 50 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 50

### INSTALL DEV PKGS

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        build-essential \
        gfortran \
        curl \
        make \
        git \
        gnupg2 \
        file \
        less \
        python3 \
        python3-pip \
        python3-dev \
        python3-distutils \
        python3-setuptools \
        tar \
        vim \
        wget \
        httpie \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

### SETUP ENTRYPOINT
COPY /ros_setup.bash /ros_setup.bash
ENTRYPOINT ["/ros_setup.bash"]
WORKDIR /
