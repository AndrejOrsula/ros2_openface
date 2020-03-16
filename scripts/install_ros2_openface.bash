## Installation DIR; TODO: Configure based on your needs
INSTALL_DIR=ros2_openface

## Create installation dir
mkdir -p ${INSTALL_DIR}/src

## Enter installation dir
cd ${INSTALL_DIR}

## Get absolute path to installation dir
INSTALL_DIR=${PWD}

## Enter source dir
cd src

## Clone OpenFace_2.1.0
git clone --depth 1 --single-branch https://github.com/TadasBaltrusaitis/OpenFace -b OpenFace_2.2.0

## Enter OpenFace dir
cd OpenFace

## Download models
bash ./download_models.sh

## Install dependencies
sudo apt-get -y install build-essential gcc-8 g++-8 zip libopenblas-dev liblapack-dev libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libtbb2 libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

## Install OpenCV 4.1.2; TODO: Enable/Disable based on your needs
./install_opencv_4_1_2.bash

## Install Dlib 19.13; TODO: Enable/Disable based on your needs
./install_dlib_9_13.bash

### Install OpenFace
## Return back to Openface dir
cd ${INSTALL_DIR}/src/Openface

## Create build folder and enter it
mkdir -p build && cd build

## Compile
cmake -D CMAKE_CXX_COMPILER=g++-8 -D CMAKE_C_COMPILER=gcc-8 -D CMAKE_BUILD_TYPE=RELEASE ..
make -j`nproc`

## Install
sudo make install
### ~ Install OpenFace

## Clone ROS 2 repos
cd ${INSTALL_DIR}/src
git clone https://github.com/AndrejOrsula/ros2_openface.git
git clone https://github.com/AndrejOrsula/vision_opencv.git -b eloquent # Version with OpenCV 4

## Install ros2_openface (ROS 2)
cd ..
touch OpenFace/COLCON_IGNORE
colcon build --symlink-install

## Add source prototype to '~/.bashrc'
echo -e "\n### ROS2 (${ROS_DISTRO}) OpenFace" >> ~/.bashrc
echo -e "# source ${INSTALL_DIR}/install/local_setup.bash" >> ~/.bashrc

