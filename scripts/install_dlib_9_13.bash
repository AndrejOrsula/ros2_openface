## Installation DIR; TODO: Configure based on your needs
INSTALL_DIR=dlib

## Create install dir
mkdir -p ${INSTALL_DIR}

## Get Dlib
wget http://dlib.net/files/dlib-19.13.tar.bz2
tar xf dlib-19.13.tar.bz2 -C ${INSTALL_DIR}
rm -r dlib-19.13.tar.bz2

## Enter install dir
cd ${INSTALL_DIR}

## Create build folder and enter it
mkdir -p build && cd build

## Build and Install
cmake ..
cmake --build . --config Release
sudo make install
sudo ldconfig
