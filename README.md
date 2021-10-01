# Differential-drive mobile robot using ROS

## Introduction
 
 A project for a differential-drive mobile robot, using ROS. 
 
 The robot has 2 back wheels and 1 fron castor wheel. Also, it is equiped with 5 sonars and 1 IMU 9-dof, measuring linear acceleration for each axis and rotation angle.
 
 ## 1st Task: Wall Follow
 
 The robot is going to start from the center of 1.5X1.5 space and moving parallel to the walls with a   desired distance. 


 ## 2nd Task: Localization
 
 The robot is going to move randomly inside of the 1.5X1.5 space. We are going to localize its position with a Discrete Kalman Filter.
 
 
 ## 3rd Task: Path Planning
 
 The robot will start from a initial node and move to the goal node, avoiding an rectangular obstacle at the centre of the space.
 
## Execution

### 1st Task: Wall Follow
```
roslaunch read_sonars read_sonars.launch
 ```
 ```
roslaunch dc_motor_driver dc_motor_driver.launch
 ```
 ```
roslaunch robo2_mobile robo2_mobile.launch
  ```
