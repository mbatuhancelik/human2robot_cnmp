#!/bin/sh

git clone https://github.com/intel-ros/realsense
cd realsense
git checkout 2.1.4
patch -p0 < ../realsense_tag_2.1.4.patch
cd ..
mv realsense ../../
sudo apt-get install -y ros-kinetic-rgbd-launch

