#!/usr/bin/env python3

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


def calc_distance_orientation(x1, x2):
    # Law of Cosines
    x3 = np.sqrt(x1**2 + x2**2 - np.sqrt(2)*x1*x2)

    # orient = orientation with respect to the wall
    theta  = np.arccos((x1**2 + x3**2 - x2**2) / (2 * x1 * x3))
    orient =  np.pi/2 - theta

    # d = mid range of triangle formed by the wall the fright and right radar beams
    d = x1 * np.sin(theta)

    return d, orient


class PID:
    def __init__(self, Kp, Ki, Kd, init_error = 0, init_time = -1):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.last_time = init_time
        self.last_error = 0
        self.proportional, self.integral, self.derivative = 0 , 0 , 0

    def __call__(self, error, time):
        dt = (time - self.last_time) 

        self.proportional = self.Kp * error
        self.integral = (self.integral + self.Ki * error * dt) 
        self.derivative = self.Kd * (error- self.last_error) / dt 

        self.last_time = time
        self.last_error = error

        return self.proportional + self.integral + self.derivative

class Logger:
    def __init__(self, logged_values):
        self.pubs = dict()
        for name in logged_values:
            self.pubs[name] = rospy.Publisher (name, Float64, queue_size=10)
    def publish(self,tbp):
        for (name,value) in tbp:
            self.pubs[name].publish(value)

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        
        # joints' states
        self.joint_states = JointState()
        
        #States
        self.prev_state = 0
        self.state = 0

        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        #PIDs

        self.linear_control = PID(1, 0 , 2)
        self.turn_control = PID(3, 0,8)
        self.angular_control = PID(20, 0, 8)
        self.turnback_control = PID(3, 0, 20)

        #Setpoints
        self.wall_min_dist_find = 0.3
        self.wall_min = 0.4
        self.find_wall_error_thres = 0.1
        self.turn_distance_thres = 1.4
        self.adjust_thres = 0.3
        self.correction = 2
        self.turn_back_correction = 0.5
        self.break_distance = 1
        self.theta_desired = np.pi/2
        self.parking_distance = 1.65

        #Offsets
        self.linear_velocity_offset = 0.1

        
        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0 / rate
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

    
    def find_wall(self, time):
        error = self.wall_min_dist_find - min (self.sonar_F.range, self.sonar_FR.range, self.sonar_FL.range)
        self.velocity.linear.x = self.linear_velocity_offset - self.linear_control(error, time)
        if error >= - self.find_wall_error_thres:
            self.state = 1
            print("Found wall, moving on to State 1")
    
    
    def adjust(self,time):
        error = self.turn_distance_thres - self.sonar_F.range
        self.velocity.linear.x = self.linear_velocity_offset
        self.velocity.angular.z = self.turn_control(error,time)

        if error < self.adjust_thres:
            self.prev_state = 1
            if corners == 8:
                self.state = 3
                print("Adjusted, moving on to State 3")
            else:
                self.state = 2
                print("Adjusted, moving on to State 2")

    def wall_follow(self,time):
        linear_error = self.wall_min - self.sonar_F.range
        d, theta = calc_distance_orientation(self.sonar_L.range, self.sonar_FL.range)
        angular_error = theta - (self.correction * (d-self.wall_min))

        self.velocity.linear.x = - self.linear_control(linear_error, time) + self.linear_velocity_offset
        self.velocity.angular.z = self.angular_control(angular_error, time)

        L.publish([('log_err_ang', angular_error), ('log_error', d - self.wall_min), ('log_theta', theta), ('log_mu', d), ('log_state2_time', time)])

        if linear_error > - 0.5:
            # Update states for countering corners #
            self.prev_state = 2
            self.state = 1
            print("Found wall, moving to State 1")

    def slow_down(self, time):

        d, theta = calc_distance_orientation(self.sonar_L.range, self.sonar_FL.range)
        angular_error = theta - (self.correction * (d-self.wall_min))
        linear_error = self.break_distance - self.sonar_F.range


        self.velocity.linear.x = - self.linear_control(linear_error, time)
        self.velocity.angular.z = self.angular_control(angular_error, time)

        ang = angular_error > - 0.01 and angular_error < 0.01
        lin = linear_error > - 0.01 and linear_error < 0.01
        if ang and lin:
            
            self.prev_state = 3
            self.state = 4
            print("Stopped, moving to State 4")

    def turn_back(self, time):

        _, theta = calc_distance_orientation(self.sonar_L.range, self.sonar_FL.range)

        error = (theta-self.theta_desired) - (self.turn_back_correction * (self.sonar_F.range-self.wall_min))

        self.velocity.angular.z = self.turn_control(error,time)

        if error < 0.01 and error > -0.01:
            self.prev_state = 4
            self.state = 5
            print("Adjusted, moving on to State 5")

    def back(self, time):

        error = self.parking_distance - self.sonar_F.range
        self.velocity.linear.x = - self.linear_control(error, time)
        print("Robot parked, end!!")
       
    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()

        print("The system is ready to execute your algorithm...")

        self.pub_rate.sleep()

        # Counter for Corners 
        global corners
        corners = 0
        
        while not rospy.is_shutdown():

            # Define time #
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()

            if self.state == 0:
                self.find_wall(time_now)

            elif self.state == 1:

                # if state change from 2->1 then a corner is detected #
                if self.prev_state == 2:
                    corners+=1
                    self.prev_state = 0
                   # print(f'Corner: {corners}')
                self.adjust(time_now)

            elif self.state == 2:
                self.wall_follow(time_now)
                
            elif self.state == 3:
                self.slow_down(time_now)

            elif self.state == 4:
                self.turn_back(time_now)

            else:
                self.back(time_now)

            # Publish the new joint's angular positions
            self.velocity_pub.publish(self.velocity)

            L.publish([('log_front', self.sonar_F.range),
                       ('log_fright', self.sonar_FR.range),
                       ('log_fleft', self.sonar_FL.range),
                       ('log_right', self.sonar_R.range),
                       ('log_left', self.sonar_L.range),
                       ('log_lin_vel', self.velocity.linear.x),
                       ('log_ang_vel', self.velocity.angular.z),
                       ('log_state', self.state),
                       ('log_time', time_now)])

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

def save_var(msg, name):
    global data_vars
    data_vars[name].append(msg.data)

if __name__ == '__main__':

    channels  = ['log_err_ang', 'log_error', 'log_lin_vel', 'log_time', 'log_ang_vel', 'log_state', 'log_theta', 'log_mu', 'log_state2_time', 'log_front', 'log_fright', 'log_fleft', 'log_right', 'log_left']
    L = Logger(channels)
    data_vars = {name: [] for name in channels}

    
    rospy.Subscriber('log_err_ang', Float64, lambda msg: save_var(msg, 'log_err_ang'))
    rospy.Subscriber('log_error', Float64, lambda msg: save_var(msg, 'log_error'))
    rospy.Subscriber('log_lin_vel', Float64, lambda msg: save_var(msg, 'log_lin_vel'))
    rospy.Subscriber('log_time', Float64, lambda msg: save_var(msg, 'log_time'))
    rospy.Subscriber('log_ang_vel', Float64, lambda msg: save_var(msg, 'log_ang_vel'))
    rospy.Subscriber('log_state', Float64, lambda msg: save_var(msg, 'log_state'))
    rospy.Subscriber('log_theta', Float64, lambda msg: save_var(msg, 'log_theta'))
    rospy.Subscriber('log_mu', Float64, lambda msg: save_var(msg, 'log_mu'))
    rospy.Subscriber('log_state2_time', Float64, lambda msg: save_var(msg, 'log_state2_time'))
    rospy.Subscriber('log_front', Float64, lambda msg: save_var(msg, 'log_front'))
    rospy.Subscriber('log_fright', Float64, lambda msg: save_var(msg, 'log_fright'))
    rospy.Subscriber('log_fleft', Float64, lambda msg: save_var(msg, 'log_fleft'))
    rospy.Subscriber('log_right', Float64, lambda msg: save_var(msg, 'log_right'))
    rospy.Subscriber('log_left', Float64, lambda msg: save_var(msg, 'log_left'))

    
    
    try:
        follower_py()
        for name in channels:
            np.save(name, data_vars[name])
    except rospy.ROSInterruptException:
        for name in channels:
            np.save(name, data_vars[name])
