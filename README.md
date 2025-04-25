# ROS_Tracer: 包含所有Tracer and velodyne VLP-16模擬和實做

## 1. 下載有問題的話可以參考以下原始連結 or ask chatGPT

```
https://github.com/agilexrobotics/ugv_gazebo_sim.git
https://github.com/ros-drivers/velodyne.git
https://github.com/lmark1/velodyne_simulator.git
```

## 2. 可以參考以下步驟

​### development environment is: Ubuntu 20.04 + ROS noetic 

Download and install ros-control function package, ros-control is the robot control middleware provided by ROS

```
sudo apt-get install ros-noetic-ros-control
```

​ Download and install ros-controllers function package, ros-controllers are the kinematics plug-in of common models provided by ROS

```
sudo apt-get install ros-noetic-ros-controllers
```

​ Download and install gazebo-ros function package, gazebo-ros is the communication interface between gazebo and ROS, and connect the ROS and Gazebo

```
sudo apt-get install ros-noetic-gazebo-ros
```

​ Download and install gazebo-ros-control function package, gazebo-ros-control is the communication standard controller between ROS and Gazebo

```
sudo apt-get install ros-noetic-gazebo-ros-control
```

Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```
sudo apt-get install ros-noetic-joint-state-publisher-gui 
```

​ Download and install teleop-twist-keyboard function package, telop-twist-keyboard is keyboard control function package, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
sudo apt-get install ros-noetic-teleop-twist-keyboard 
```

## 3. 如何使用

### create workspace

### 注意：如果以下方式有問題，請參考上述下載網址自行下載，這個kokroiceork75的README是我自己寫的，所以下載可能有錯，但檔案是沒問題的，所以參考以上下載連結後，再把裡面的檔案替換成這個GitHub的檔案

### bashrc可以source，以防每次開terminal都要source一次

```
mkdir tracer_ws
cd tracer_ws
mkdir src
cd src
catkin_init_workspace
git clone this url
cd tracer_ws
rosdep install --from-paths src --ignore-src -r -y 
catkin_make
source devel/setup.bash
```
