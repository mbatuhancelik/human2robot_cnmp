
# torobo_demo package
The `torobo_demo` package provides demo programs.

## How to use
Before launch torobo_demo, activate torobo_robot model's simulation by following command:
```
$ roslaunch torobo_bringup torobo_bringup.launch sim:=true
```

Then, execute demo programs by the following command in another terminal.

### demo moving the arm
```
$ rosrun torobo_demo arm_follow_trajectory.py
```
Thie sample program moves the arm by ROS's control_msg action 'Follow Joint Trajectory'.

### demo moving the gripper by action of ROS's control_msg
```
$ rosrun torobo_demo arm_gripper_command.py
```
Thie sample program moves the arm by ROS's control_msg action 'Gripper Command'.

### demo performing moveit commander
```
$ rosrun torobo_demo arm_moveit_commander.py
```
This sample program moves the arm by using moveit commander (motion planning).

### demo performing moveit kinematics
```
$ rosrun torobo_demo arm_moveit_kinematics.py
```
This sample program calculates forward/inverse kinematics of torobo_robot.

### demo performing torobo_msgs
```
$ rosrun torobo_demo arm_torobo_msgs.py
```
This sample program call ros services for real torobo_robot.

### demo performing easy_command
```
$ rosrun torobo_demo arm_easy_command.py
```
This sample program call torobo_easy_command for sending command string to controller box.

