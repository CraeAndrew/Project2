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
        
        self.x = 0.0
        self.y = 0.0

        self.theta = 0.0

        self.publisher = self.create_publisher(PoseStamped, 'vehicle_pose', 10)
       
    def listener_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def listener_callback2(self, msg):

        v = msg.twist.twist.linear.x

        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        self.theta = 2*math.atan2(z, w)

        x_calc = self.x + v * math.cos(self.theta) * 0.15
        y_calc = self.y + v * math.sin(self.theta) * 0.15
        
        msgEst = PoseStamped()
        msgEst.pose.position.x = x_calc
        msgEst.pose.position.y = y_calc
        msgEst.pose.orientation.z = z
        msgEst.pose.orientation.w = w

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
