import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from std_msgs.msg import Int16
from std_msgs.msg import Float32

#from drive_interfaces.msg import VehCmd

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

import math

class VehiclePose(Node):

    def __init__(self):
        super().__init__('VehPose')
        
        self.subscription = self.create_subscription(
            PoseStamped,'GPSData',self.listener_callback,10)
        
        self.subscription2 = self.create_subscription(
            Odometry,'odometry',self.listener_callback2,10)   

        self.subscription  # prevent unused variable warning
        self.subscription2
        
        self.x = 0
        self.y = 0

        self.theta.z = 0
        self.theta.w = 0
        #self.ov = 0
        #self.ow = 0

        self.publisher = self.create_publisher(PoseStamped, 'vehicle_pose', 10)
       
    def listener_callback(self, msg):
        
        delta_t = 0.15
        

    def listener_callback2(self, msg):
        

        
        msgEst = PoseStamped()
        msgEst.pose.position.x = 1
        msgEst.pose.position.y = 1
        msgEst.pose.orientation.z =  # pass in the odometry pose.pose.orientation.z
        msgEst.pose.orientation.w =  1 # pass in the odometry pose.pose.orientation.w
        
        self.theta.z = 2*math.atan(2*z)
        self.theta.w = 2*math.atan(2*w)
        
        self.publisher.publish(msgEst)

def main(args=None):
    rclpy.init(args=args)

    VehPose = VehiclePose()

    rclpy.spin(VehPose)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    VehPose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
