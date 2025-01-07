# InGBVIO
A tightly coupled and invariant GNSS-Barometer-Visual-Inertial Odometry

update on 01-07-2025

GBVIO dataset link:  https://cloud.tsinghua.edu.cn/d/1b81eac00eb643aaa395/

The source code is still in the process of being organized.


## 1. System Requirements

### 1.1  Support of C++ 14 Features

The compiler should at least support c++14 standards.

### 1.2  ROS-Noetic System

InGVIO is developed under [ROS-Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

### 1.3  Eigen Library

InGVIO is developed under [Eigen3.3.7](https://eigen.tuxfamily.org/index.php?title=Main_Page).

### 1.4  SuiteSparse Library

We use [SuiteSparse](https://github.com/DrTimothyAldenDavis/SuiteSparse/releases) Library for sparse QR-decomposition in visual updates. 

### 1.5  gnss_comm Library

A wrapper for GNSS messages in ROS. See [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm).

### 1.6  irp_sen_msg library

A Ros message library for Kaist Urban dataset. Clone this [msg package](https://github.com/tsyxyz/irp_sen_msgs) to the src folder of catkin_ws.

## 2. Build InGBVIO

Download or clone this repo and [irp_sen_msg](https://github.com/tsyxyz/irp_sen_msgs) to your ROS workspace.

```
cd ~/ws_catkin
catkin_make
```

Source the setup file to let ROS recognize the related launch files.

```
source devel/setup.bash
```
