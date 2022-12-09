import rclpy
from rclpy.node import Node

import math
import numpy as np
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from gps_nav_interfaces.msg import CurrentGoalPose
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from drive_interfaces.msg import VehCmd   # must be same message format as av1tenth drive_interfaces
from gps_nav.uf_support.geometry_support import value_near

D2R = math.pi/180.0
R2D = 180.0/math.pi

class VehicleControllerPointAtCarrot(Node):

    def __init__(self):
        super().__init__('vehicle_controller')
        self.subscription1 = self.create_subscription(
            PoseStamped,'vehicle_pose', self.vehicle_pose_callback, 1)
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            CurrentGoalPose, 'current_goal_pose', self.current_goal_pose_callback, 1)
        self.subscription2  # prevent unused variable warning

        self.subscription3 = self.create_subscription(
            Int8, 'e_stop', self.e_stop_callback, 1)
        self.subscription3  # prevent unused variable warning

        self.publisher = self.create_publisher(VehCmd, 'vehicle_command_angle', 10)
        self.publisher2 = self.create_publisher(Twist, 'vehicle_command_twist', 10)

        # set up the timer (0.1 sec) to send over the current_carrot message to the vehicle controller
        self.main_timer = self.create_timer(0.1, self.main_timer_callback)

        self.declare_parameter('L_wheelbase', 0.3)  # meters
        self.declare_parameter('K_p', 0.2)  # PID parameter
        self.declare_parameter('K_d', 0.1)  # PID parameter
        self.declare_parameter('K_i', 0.0)  # PID parameter
        self.K_p = self.get_parameter('K_p').get_parameter_value().double_value
        self.K_d = self.get_parameter('K_d').get_parameter_value().double_value
        self.K_i = self.get_parameter('K_i').get_parameter_value().double_value

        self.wheelbase = self.get_parameter('L_wheelbase').get_parameter_value().double_value

        # define the variables that will store the data from the two message inputs
        self.current_goal_point = np.array([0.0, 0.0, 0.0])
        self.current_goal_heading_rad = 0.0
        self.closest_point = np.array([0.0, 0.0, 0.0])
        self.closest_heading_rad = 0.0
        self.speed = 0.0
        self.state = 0.0
        self.goal_cnt = 0

        self.vehicle_point = np.array([0.0, 0.0, 0.0])
        self.vehicle_heading_rad = 0.0
        self.vehicle_pose_cnt = 0

        self.pause = False
        
        self.have_vehicle_pose = False
        self.have_goal_pose = False

        self.old_steering_angle = 0.0
        self.prev_error = 0.0
        self.t_current = 0
        self.t_previous = 0
        self.error_integral = 0.0

        self.xvec = np.array([1.0, 0.0, 0.0])
        self.yvec = np.array([0.0, 1.0, 0.0])
        self.zvec = np.array([0.0, 0.0, 1.0])

        self.error1 = 0.0
        self.error2 = 0.0
        
    def vehicle_pose_callback(self, msg):
        self.have_vehicle_pose = True

        self.vehicle_point = np.array([0.0,0.0,0.0])
        self.vehicle_point[0] = msg.pose.position.x
        self.vehicle_point[1] = msg.pose.position.y
        self.vehicle_point[2] = 0.0
        self.vehicle_heading_rad = 2.0*math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

        self.vehicle_pose_cnt += 1

    def current_goal_pose_callback(self, msg): 
        self.have_goal_pose = True

        self.current_goal_point = np.array([0.0,0.0,0.0])
        self.current_goal_point[0] = msg.current_goal_pose.pose.position.x
        self.current_goal_point[1] = msg.current_goal_pose.pose.position.y
        self.current_goal_point[2] = 0.0
        self.current_goal_heading_rad = 2.0*math.atan2(msg.current_goal_pose.pose.orientation.z, msg.current_goal_pose.pose.orientation.w)

        self.closest_point = np.array([0.0,0.0,0.0])
        self.closest_point[0] = msg.closest_pose.pose.position.x
        self.closest_point[1] = msg.closest_pose.pose.position.y
        self.closest_point[2] = 0.0
        self.closest_heading_rad = 2.0*math.atan2(msg.closest_pose.pose.orientation.z, msg.closest_pose.pose.orientation.w)

        self.speed = msg.speed
        self.state = msg.state

        self.goal_cnt += 1

    def e_stop_callback(self, msg):
        if (msg.data == 0):
            self.pause = True
        elif (msg.data == 1):
            self.pause = False

    def main_timer_callback(self):
        # will only publish after each of the subscribes has occured at least once

        if (self.pause == True):

            # send out a zero velocity twist
            out_msg = VehCmd()
            out_msg.steering_angle = 0.0
            out_msg.throttle_effort = 0.0

            self.publisher.publish(out_msg)

        elif (self.have_goal_pose and self.have_vehicle_pose):
            
            # Put your controller below here.
            self.error = 0.0
            self.error = math.sqrt((self.closest_point[0]-self.vehicle_point[0])^2+(self.closest_point[1]-self.vehicle_point[1])^2)
            new_steering_angle = self.old_steering_angle + self.K_p*(self.closest_heading_rad - self.self.vehicle_heading_rad) + self.K_p*self.error
            self.old_steering_angle = new_steering_angle
            
            # now send a VehCmd message out
            out_msg = VehCmd()
            out_msg.steering_angle = new_steering_angle
            if (self.speed < 0.01):
                out_msg.throttle_effort = 0.0
            else:
                out_msg.throttle_effort = 50.0  # percent
            self.publisher.publish(out_msg)
            
def main(args=None):
    rclpy.init(args=args)

    my_vehicle_controller = VehicleControllerPointAtCarrot()

    rclpy.spin(my_vehicle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_vehicle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
