cmake_minimum_required(VERSION 2.8.3)
project(torobo_gripper_action_controller)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin 
  REQUIRED COMPONENTS
      actionlib
      angles
      cmake_modules
      controller_interface
      controller_manager
      control_msgs
      control_toolbox
      hardware_interface
      realtime_tools
      roscpp
      trajectory_msgs
      urdf
      xacro
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS 
  actionlib 
  cmake_modules 
  controller_interface 
  control_msgs 
  hardware_interface 
  realtime_tools
  trajectory_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/gripper_action_controller.cpp
   include/torobo_gripper_action_controller/gripper_action_controller.h
)

target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES ros_control_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
