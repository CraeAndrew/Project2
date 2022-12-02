import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from std_msgs.msg import Int16
from std_msgs.msg import Float32

from drive_interfaces.msg import VehCmd

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

import math

class AutoVeh(Node):

    def __init__(self):
        super().__init__('VehPose')
        
        self.subscription = self.create_subscription(
            PoseStamped,'vehicle_pose',self.listener_callback,10)
        
        self.subscription2 = self.create_subscription(
            PoseStamped,'current_goal_pose',self.listener_callback2,10)   

        self.subscription  # prevent unused variable warning
        self.subscription2
        
        self.publisher = self.create_publisher(VehCmd, 'vehicle_command_angle', 10)
       
    def listener_callback(self, msg):
        a=0
    def listener_callback2(self, msg):
        b=0
        #self.publisher.publish()

def main(args=None):
    rclpy.init(args=args)

    avp = AutoVeh()

    rclpy.spin(avp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
