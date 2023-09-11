# Change Log

## v2.8.0 (2020-3-26)

### Fixed

- Fixed the spawner.py script in package torobo_control due to an error caused by a change in the SwitchControllerService specification.
- Changed to use "safe_dump" because the update of the related package causes an error when dumping YAML in torobo_movei_config package.


## v2.7.1 (2020-3-11)

### Changed

- Changed camera_link_tf.yaml in torobo_camera package.

## v2.7.0 (2020-2-11)

### Changed

- Changed root link detection method in rbdl package.

### Fixed

- Fixed some test bugs in torobo_dynamics, torobo_control, torobo_motion_manager and torobo_test packages.

## v2.6.0 (2019-12-12)

### Fixed

- Fixed the problem that current_state_self_collision_checker does not work when waitForServiceExistence fails in torobo_collision_detector package.

## v2.5.1 (2019-12-6)

### Changed

- Changed camera_link_tf.yaml in torobo_camera package.

## v2.5.0 (2019-9-23)

### Added

- Add demo scripts *_publish_joint_trajectory.py and *_publish_gripper_command.py to torobo_demo.

### Fixed

- Fixed the problem where MoveTeachingPoint/Trajectory/Home action in torobo_motion_manager would rarely cause segmentation fault.

## v2.4.0 (2019-8-28)

### Changed

- Changed start point deviation judgment in torobo_driver to be based on the difference of each joint from the sum of the differences of each joint.

## v2.3.0 (2019-7-2)

### Added

- Add GetCollisionInfo.srv to torobo_msgs and implemented the same service server in torobo_collision_detector.

### Changed

- Add joint/link_tip to base URDF and change each robots URDF parent reference.

## v2.2.1 (2019-6-4)

### Changed

- Replace DAE mesh of TRBA-R and TRBA-M with colored ones.
- Replace STL mesh of TRBA-R and TRBA-M with fattened ones.

## v2.2.0 (2019-5-8)

### Fixed

- Fixed the problem of mismatching images when using Intel RealSense in Gazebo/Rviz.

## v2.1.0 (2019-4-9)

### Added

- Added tolerance parameters to controllers.yaml in torobo_description.

### Changed

- Update libmcc version to v2.1.0.
    - Changed resend timeout threshold value when reception confirmation fails from 500 ms to 100 ms.

### Fixed

- Fixed the problem of operation delay of torobo_whole_body_manager.py in torobo_gui.

## v2.0.0 (2019-3-22)

### Added

- Added a function of jog control to torobo_joint_controller.py in torobo_gui.

### Changed

- Refactored whole package.
- Change IP address settings of torobo_controller_config.yaml in torobo_driver: 192.168.0.XXX -> 10.10.253.XXX

## v1.6.0 (2019-1-10)

### Changed
- Added a function to automatically set gravity vector according to URDF in torobo_dynamics

### Fixed
- Fix a bug in torobo_easy_command.py in torobo_driver: When sending commands to multiple controllers, errors are displayed on controllers that are not originally intended.

## v1.5.0 (2018-11-26)

### Added
- Add torobo_gazebo_ros_control package in order to avoid SetPosition jump problem of prismatic joint
- Add torobo_gripper_action_controller package in order to avoid crash when preempting gripper action

### Changed
- Change ros_control plugin from gazebo_ros_control to torobo_gazebo_ros_control written on torobo_description/urdf/torobo.gazebo
- Change gripper_controller's type from GripperActionController to ToroboGripperActionController written on torobo_description/config/.../controllers.yaml

### Fixed
- Fix a bug in ToroboJointController: '<New>' item in TP list is removed when 'Enter' key is pressed when '<New>' item is selected in TP list. 

## v1.4.0 (2018-10-29)
### Changed
- Perform refactoring of the function to load rosparam dynamically at initialization
- Separate URDF of base from robot's URDF
