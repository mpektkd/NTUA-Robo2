# Differential-drive mobile robot using ROS

## Introduction
 
 A project for a differential-drive mobile robot, using ROS. 
 
 The robot has 2 back wheels and 1 fron castor wheel. Also, it is equiped with 5 sonars and 1 IMU 9-dof, measuring linear acceleration for each axis and rotation angle.
 
 ## Wall Follow
 
 The robot is going to start from the center of 1.5X1.5 space and moving parallel to the walls with a   desired distance. 
 
## Execution

### Wall Follow
```
roslaunch read_sonars read_sonars.launch
 ```
 ```
roslaunch dc_motor_driver dc_motor_driver.launch
 ```
 ```
roslaunch robo2_mobile robo2_mobile.launch
  ```
