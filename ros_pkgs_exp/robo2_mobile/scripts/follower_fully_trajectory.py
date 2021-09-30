#!/usr/bin/env python

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import matplotlib.pyplot as plt
import os
import numpy as np
import time as t

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('sonar_front', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('sonar_front_left', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('sonar_front_right', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('sonar_left', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('sonar_right', Range, self.sonar_right_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)

    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        ###########################################################################################################
        ############################################ START OF OUR CODE ############################################
        ###########################################################################################################

        # PARAMETERS FOR PROBLEM #
        front_distance = 0.2           # disered front distance from obstacle
        side_distance = 0.4            # disered side distance from obstacle

        #################### VARIABLES FOR PD-CONTROLER ####################
        #Kp = 15                         # parameters for PD-Controler
        #Kd = 13
        #
        #conv_param = 2                  # correlation parameter
        ####################################################################

        # varibles used for simulation
        start_time = rostime_now.to_sec()   # define start time of simulation || used to print graphs
        print_time = 80.0                   # used to print graphs
        print_dis = 0
        hide_display = 0
        start = True                        # flag for the initialization
        stop = False                        # flag for stoping the movement

        t = 0                               # variable to keep track of time steps
        init_1 = 0                          # flag to re-initialize t
        init_2 = 0                          # flag to re-initialize t

        #################### VARIABLES FOR PD-CONTROLER ####################
        #error = 0                          # error of the measurement 
        ####################################################################       
        
        # lists to append elements for graphs
        dis_0 = []
        linear_0 = [] 
        angular_0 = []
        time_0 = []

        dis_1 = []
        linear_1 = [] 
        angular_1 = []
        time_1 = []
        
        dis_2 = []
        linear_2 = [] 
        angular_2 = []
        time_2 = []


        while not rospy.is_shutdown():
           
            sonar_front = self.sonar_F.range      # get measurement from front sensor
            sonar_frontleft=self.sonar_FL.range   # get measurement from front-left sensor   
            sonar_frontright=self.sonar_FR.range  # get measurement from front-right sensor    
            sonar_left=self.sonar_L.range         # get measurement from left sensor   
            sonar_right=self.sonar_R.range        # get measurement from right sensor

            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)

            running_time = rostime_now.to_sec() - start_time    # current time of simulation

            # create the graphs 
            if ((running_time > print_time) and (hide_display == 0) and (print_dis == 1)):

                if not os.path.isdir('graphs'):
                    os.mkdir('graphs')          # Create directory if it doesn't allready exists

                # print("printing graphs in {}/graphs".format(os.path.abspath(os.path.dirname(__file__)))) # directory to save graphs 
                hide_display = 1 

                # Calculate and print total time of simulation
                minutes=int(running_time/60)
                seconds=running_time-minutes*60
                # print("Total time of simulation: {} minutes {} seconds".format(minutes,seconds))

                # graph for distance error
                # plt.figure(figsize=(20,6))
                # plt.plot(time_0,dis_0, color="red", marker='o', linestyle='None')
                # plt.plot(time_1,dis_1, color="blue", marker='o', linestyle='None')
                # plt.plot(time_2,dis_2, color="green", marker='o', linestyle='None')
                # plt.legend(["State 0", "State 1", "State-2"])
                # plt.xlabel('Time (sec)')
                # plt.ylabel('Error (cm)')
                # plt.title('Distance error from wall')
                # plt.savefig("graphs/dis_err.png")
                # plt.close()
                
                # # graph for distance error (state-1 & state-2)
                # plt.figure(figsize=(20,6))
                # plt.plot(time_1,dis_1, color="blue", marker='o', linestyle='None')
                # plt.plot(time_2,dis_2, color="red", marker='o', linestyle='None')
                # plt.legend(["State 1", "State-2"])
                # plt.xlabel('Time (sec)')
                # plt.ylabel('Error (cm)')
                # plt.title('Distance error from wall')
                # plt.savefig("graphs/dis12_err.png")
                # plt.close()

                # # graph for linear velocity
                # plt.figure(figsize=(20,6))
                # plt.plot(time_0,linear_0, color="red", marker='o', linestyle='None')
                # plt.plot(time_1,linear_1, color="blue", marker='o', linestyle='None')
                # plt.plot(time_2,linear_2, color="green", marker='o', linestyle='None')
                # plt.legend(["State 0", "State 1", "State 2"])
                # plt.xlabel('Time (sec)')
                # plt.ylabel('Linear Velocity (cm/s)')
                # plt.title('Linear Velocity of the robotic vehicle')
                # plt.savefig("graphs/lin_vel.png")
                # plt.close()

                # # graph for angular velocity
                # plt.figure(figsize=(20,6))
                # plt.plot(time_0,angular_0, color="red", marker='o', linestyle='None')
                # plt.plot(time_1,angular_1, color="blue", marker='o', linestyle='None')
                # plt.plot(time_2,angular_2, color="green", marker='o', linestyle='None')
                # plt.legend(["State 0", "State 1", "State 2"])
                # plt.xlabel('Time (sec)')
                # plt.ylabel('Angular Velocity (rad/s)')
                # plt.title('Angular Velocity of the robotic vehicle')
                # plt.savefig("graphs/ang_vel.png")
                # plt.close()

            # Stop Vehicle
            if (running_time > print_time):
                print_dis = 1
                linear_vel = 0.0
                angular_vel = 0.0
                stop = True    
                        
            # Calculate angle and distance to perform th PD-Control
            # x1 = sonar_left
            # x2 = sonar_frontleft
            x1 = sonar_right
            x2 = sonar_frontright
            x3 = np.sqrt(x1**2 + x2**2 - np.sqrt(2)*x1*x2)
            angle0 = np.arccos((x1**2 + x3**2 - x2**2) / (2*x1*x3))
            ##### CODE FOR PD-CONTROL #####
            #angle = np.pi/2 - angle0 
            ###############################
            distance = x1*np.sin(angle0)
            
            ################ CODE FOR PD-CONTROL ################
            #prev_error = error
            #error = angle - conv_param*(distance-side_distance)
            #####################################################


            if (not stop):
                # STATE 0: linear movement until wall is found
                if start:
                    if (min(sonar_front, sonar_frontleft, sonar_frontright) > front_distance):
                        if (t <= 0.5):
                                linear_vel = 6*t**2 - 8*t**3
                                angular_vel = 0.0
                                t = t + 0.1	

                        else:
                                linear_vel = 0.5
                                angular_vel = 0.0      

                        # append values into the lists for the graph display
                        time_0.append(running_time)
                        dis_0.append(np.abs(distance-side_distance))
                        linear_0.append(linear_vel)
                        angular_0.append(angular_vel)              

                    else:
                        front_distance = 0.3
                        start = False	
                        t = 0


                else:
                    # STATE 2: linear and angular movement while robot is driven parallel to the wall
                    if (min(sonar_front, sonar_frontleft, sonar_frontright) > front_distance):
                        if (init_2 ==1):
                            t = 0
                            init_2 = 0

                        if (t <= 0.5):
                                linear_vel = 6*t**2 - 8*t**3
                                angular_vel = 0.5 - 6*t**2 + 8*t**3
                                t = t + 0.1	

                        else:
                                linear_vel = 0.5
                                angular_vel = 0
                                init_1 = 1

                        ################## CODE FOR PD-CONTROL ####################
                        #angular_vel = Kp*error + Kd*(error - prev_error)/max(1,dt) 
                        ###########################################################

                        # append values into the lists for the graph display
                        time_2.append(running_time)
                        dis_2.append(np.abs(distance-side_distance))
                        linear_2.append(linear_vel)
                        angular_2.append(angular_vel)                       

                    else:
                        # STATE 1: linear and angular movement until the robot is turned
                        if (init_1 ==1):
                            t = 0
                            init_1 = 0

                        if (t <= 0.5):
                            linear_vel = 0.5 - 6*t**2 + 8*t**3
                            angular_vel = 6*t**2 - 8*t**3
                            t = t + 0.1	

                        else:
                            linear_vel = 0.0
                            angular_vel = 0.5
                            init_2 = 1

                        # append values into the lists for the graph display
                        time_1.append(running_time)
                        dis_1.append(np.abs(distance-side_distance))
                        linear_1.append(linear_vel)
                        angular_1.append(angular_vel)

            # Change the velocity of the vehicle
            self.velocity.linear.x = linear_vel
            self.velocity.angular.z = angular_vel    

        ###########################################################################################################
        ############################################# END OF OUR CODE #############################################
        ###########################################################################################################
                
            ## Calculate time interval (in case is needed)
            #time_prev = time_now
            #rostime_now = rospy.get_rostime()
            #time_now = rostime_now.to_nsec()
            #dt = (time_now - time_prev)/1e9

            # Publish the new joint's angular positions
            self.velocity_pub.publish(self.velocity)

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
