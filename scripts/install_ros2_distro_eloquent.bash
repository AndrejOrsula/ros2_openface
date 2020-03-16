## Setup Sources
sudo apt-get update && sudo apt-get install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

## Install ROS eloquent packages
sudo apt-get update
sudo apt-get install ros-eloquent-desktop

## Install dev tools
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
python3 -m pip install -U \
  argcomplete \
  flake8 \
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
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools

## Install Fast-RTPS dependencies
sudo apt-get install --no-install-recommends -y libasio-dev libtinyxml2-dev

# Install CycloneDDS dependencies
sudo apt-get install --no-install-recommends -y libcunit1-dev

## Add source prototype to '~/.bashrc'
echo -e "\n### ROS2 (eloquent)" >> ~/.bashrc
echo -e "# source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
