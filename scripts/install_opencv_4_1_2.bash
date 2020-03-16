## Installation DIR; TODO: Configure based on your needs
INSTALL_DIR=opencv

## Install dependencies
sudo apt install -y python3-opencv build-essential cmake git pkg-config libgtk-3-dev \
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev \
libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev python3-dev \
python3-numpy libtbb2 libtbb-dev libdc1394-22-dev

## Create installation directory
mkdir -p ${INSTALL_DIR} && cd ${INSTALL_DIR}

## Update installation dir to absolute path
INSTALL_DIR=${PWD}

## Clone repositories. You can manually checkout a specific release if you want
git clone --depth 1 --single-branch https://github.com/opencv/opencv.git -b 4.1.2
git clone --depth 1 --single-branch https://github.com/opencv/opencv_contrib.git -b 4.1.2

## Crate build directory
mkdir opencv/build && cd opencv/build

## Setup CMake
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_C_EXAMPLES=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_EXTRA_MODULES_PATH=${INSTALL_DIR}/opencv_contrib/modules \
-D BUILD_EXAMPLES=ON ..

## Compile
make -j`nproc`

## Install
sudo make install
