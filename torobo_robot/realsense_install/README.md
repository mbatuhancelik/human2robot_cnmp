# HOW TO INSTALL realsense2_camera

This is a document to explain how to install realsense2_camera package manually.

## Installation

### Step 1: Install the latest librealsense2 of RealSense SDK2.0

```
git clone https://github.com/IntelRealSense/librealsense
cd librealsense
git checkout v2.18.0
```

Then build and make install librealsense2.
Details of installation is described in [librealsense/doc/installation.md](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

### Step 2: Download the realsense ROS package

```
git clone https://github.com/intel-ros/realsense
cd realsense
git checkout 2.1.4
```

### Step 3: Apply patch

```
patch -p0 < ../realsense_tag_2.1.4.patch
```

### Step 4: Move directory

```
cd ..
mv realsense ../..
```

### Step 5: Build catkin

```
cd ~/catkin_ws
catkin build
catkin source
```

### Step 6: launch ROS

```
roslaunch realsense2_camera rs_rgbd.launch
```
