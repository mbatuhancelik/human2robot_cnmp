
# Table of Contents
- [torobo_robot ROS](#toroborobot-ros)
  - [Supported versions](#supported-versions)
  - [File System](#file-system)
  - [Installation](#installation)
      - [ROS installation](#ros-installation)
      - [Dependent package installation](#dependent-package-installation)
      - [Build torobo_robot](#build-torobo_robot)
  - [How to use](#how-to-use)
      - [Visualize Torobo in rviz](#visualize-torobo-in-rviz)
      - [Motion planning simulation](#motion-planning-simulation)
      - [Demo](#demo)


# torobo_robot ROS

The `torobo_robot` provides a ROS interface for the robots of Tokyo Robotics.

## Supported versions
The recommended configuration is ROS Kinetic Kame with Ubuntu 16.04.
This package has only been tested for the one recommended above. 

## File system
- `torobo_bringup`: a package that has launch file to start torobo_robot ros applications.
- `torobo_collision_detector`: a package to detect collision.
- `torobo_common`: a package that contains parsers for YAML/SRDF.
- `torobo_control`: a package that compiles the control settings of torobo.
- `torobo_demo`: a package that has simple demo programs.
- `torobo_description`: a package that has the configuration model information of torobo.
- `torobo_driver`: a package that has driver for real-target of torobo_robot.
- `torobo_dynamics`: a package that calculates self-weight compensation and inertia-compensation.
- `torobo_gazebo`: a package that compiles various settings for using Torobo with the physical simulator gazebo.
- `torobo_gazebo_ros_control`: a workaround package for gazebo's bug (=SetPosition Function results in illegal movement on position_controller's prismatic(slider) joint).
- `torobo_gripper_action_controller`: a workaround package for ros_controllers GripperActionController's bug (crash occurs when preempting). This package is bundled only for models with gripper.
- `torobo_gui`: a package that has gui tools to control torobo_robot.
- `torobo_motion_manager`: a package that has functions for teaching trajectories.
- `torobo_moveit_config`: a package that has moveit setting files.
- `torobo_msgs`: a package that defines torobo_robot's original commands.
- `torobo_operation`: a package that has actionlib services for move home and move to teaching points.
- `torobo_resources`: a package that has robot urdf models and meshes.
- `torobo_robot`: a metapackage that put together all packages for ros.


## Installation

### ROS installation
This operation is unnecessary if ROS Kinetic Kame has already been installed.

#### Install build essentials
```
sudo apt-get update -y
sudo apt-get install -y build-essential
sudo apt-get install -y python-dev
sudo apt-get install -y python-pip
```

#### Install ROS Kinetic Kame
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://ha.pool.sks-keyservers.net:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update -y
sudo apt-get install -y ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
sudo apt-get install -y python-rosinstall
sudo apt-get install -y python-catkin-tools
sudo apt-get install -y ros-kinetic-moveit
sudo apt-get install -y ros-kinetic-moveit-visual-tools
sudo apt-get install -y ros-kinetic-gazebo-ros-control
sudo apt-get install -y ros-kinetic-ros-controllers
source ~/.bashrc
echo "source `catkin locate --shell-verbs`" >> ~/.bashrc
source ~/.bashrc
```

#### catkin_ws setup
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

### build torobo_robot
Please place the torobo_robot package under `~/catkin_ws/src/` below before doing the following operations.
The directory structure is as follows.
```
~/catkin_ws
           `--src
                 `--torobo_robot
                      |--torobo_bringup
                      |--torobo_collision_detector
                      |--torobo_common
                      |--torobo_control
                      |--torobo_demo
                      |--torobo_description
                      |--torobo_driver
                      |--torobo_dynamics
                      |--torobo_gazebo
                      |--torobo_gazebo_ros_control
                      |--torobo_gripper_action_controller (bundled only with gripper model)
                      |--torobo_gui
                      |--torobo_motion_manager
                      |--torobo_moveit_config
                      |--torobo_msgs
                      |--torobo_operation
                      |--torobo_resources
                      `--torobo_robot
```

Please build with the following command.
```
cd ~/catkin_ws
catkin build
catkin source
```

### How to use
#### Visualize Torobo in rviz
```
$ roslaunch torobo_description torobo_display.launch
```

#### Motion planning with simulation
```
$ roslaunch torobo_bringup torobo_bringup.launch sim:=true
```
> `Warning`  
> On the screen of Gazebo, gripper's finger may move illegally.  
> This is a phenomenon in which Gazebo version is 7.x or later and occurs only for prismatic joints.  
> The ROS community reports it's a known bug of Gazebo itself.  
> Since full fix has not been found, please restart Gazebo when this problem occurs.  

#### Motion planning with real robot
```
$ roslaunch torobo_bringup torobo_bringup.launch sim:=false
```

#### Demo
After activating the motion planning simulation, execute the following command in another terminal.
For more details please refer to README.md in torobo_demo package.
