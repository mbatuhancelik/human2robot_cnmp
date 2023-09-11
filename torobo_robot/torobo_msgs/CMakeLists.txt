cmake_minimum_required(VERSION 2.8.3)
project(torobo_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  CollisionInfoRequest.msg
  CollisionInfoResponse.msg
  ToroboJointState.msg
  ToroboDynamics.msg
  ToroboCommand.msg
)

add_service_files(
  FILES
  CheckCollision.srv
  DeleteTeachingPoint.srv
  DeleteTeachingTrajectory.srv
  GetCollisionInfo.srv
  GetJointState.srv
  GetServoState.srv
  GetTeachingPoint.srv
  GetTeachingPointNames.srv
  GetTeachingTrajectory.srv
  GetTeachingTrajectoryNames.srv
  GetToroboJointState.srv
  RecordTeachingPoint.srv
  RecordTeachingTrajectory.srv
  SendCommonCommand.srv
  ErrorReset.srv
  BrakeOff.srv
  BrakeOn.srv
  ServoOff.srv
  ServoOn.srv
  SetControlMode.srv
  SetPayloadParam.srv
  SetZeroEffort.srv
  ClearTrajectory.srv
  CancelTrajectory.srv
  SetRobotControllerParameter.srv
  SetGeneralOutputRegister.srv
)

add_action_files(DIRECTORY action
  FILES
  MoveHomePosition.action
  MoveTeachingPoint.action
  MoveTeachingTrajectory.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs sensor_msgs trajectory_msgs moveit_msgs message_runtime
)
