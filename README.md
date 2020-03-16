# ROS 2 Wrapper for OpenFace
![OS](https://img.shields.io/badge/OS-Ubuntu_18.04-orange.svg) ![ROS_2](https://img.shields.io/badge/ROS_2-Eloquent-brightgreen.svg) ![OPENFACE](https://img.shields.io/badge/OpenFace-2.2-lightgrey.svg)


This repository contains a ROS 2 wrapper for [OpenFace](https://github.com/TadasBaltrusaitis/OpenFace), which allows to extract the following information from an RGB video, see [msgs](ros2_openface_msgs/msg).
- 2D/3D Facial Landmarks
- Head Pose
- 2D/3D Eye Landmarks
- Gaze
  - Left Eye
  - Right Eye
  - Compound
- Action Units

## Installation
Please feel free to follow the installation scripts provided in the [`scripts` folder](scripts), namely the [`ROS 2 Eloquent` installation script](scripts/install_ros2_distro_eloquent.bash) (if not already installed) and [`ROS2 OpenFace` installation script](scripts/install_ros2_openface.bash), which encapsulates the rest.


### Requirements
These are the build/run-time requirements for this package.

#### 1) [ROS 2 Eloquent](https://index.ros.org/doc/ros2/Installation/Eloquent) with [Development Tools](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/#install-development-tools-and-ros-tools)

#### 2) [OpenFace 2](https://github.com/TadasBaltrusaitis/OpenFace/wiki/Unix-Installation) (tested with 2.2.0)
Together with its dependencies, such as [dlib 9](http://dlib.net/compile.html) (tested with 9.13) and [OpenCV 4](https://github.com/opencv/opencv) (tested with 4.1.2).

#### 3) [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2) (with support for OpenCV 4)

### Building
Build with [colcon](https://colcon.readthedocs.io/en/released).


## Usage
First, source the ROS 2 global installation (if not done before).
```bash
source /opt/ros/eloquent/setup.bash
```

Then source the ROS 2 workspace overlay (if not done before).
```bash
source /path/to/ros2_openface/install/local_setup.bash
```

Now it can be run with your favourite camera or video from ros2bag.
```bash
ros2 launch openface openface.launch.py
```


## Acknowledgment
This repository is based on the amazing work of authors that contributed to OpenFace as well as its the underlying frameworks it utilises.


## License
This project is licensed under [BSD 3-Clause License](LICENSE).

You have to respect OpenFace, dlib, and OpenCV licenses, together with licenses of the datasets used for OpenFace model training.
