import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math


class MotorCommandNode(Node):
    def __init__(self):
        super().__init__('motor_command_node')
        
        self.wheel_radius_ = 0.1
        self.wheel_separation_ = 0.46
        
        self.left_motor_rpm_pub_ = self.create_publisher(Float64, '/left_motor_rpm', 10)
        self.right_motor_rpm_pub_ = self.create_publisher(Float64, '/right_motor_rpm', 10)
        
        self.cmd_vel_sub_ = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
    
    def cmd_vel_callback(self, msg):

        vx = msg.linear.x
        wz = msg.angular.z       
        
        left_motor_rad_per_sec = (vx - (self.wheel_separation_ * wz) / 2.0) / self.wheel_radius_
        right_motor_rad_per_sec = (vx + (self.wheel_separation_ * wz) / 2.0) / self.wheel_radius_
        
        left_motor_rpm = left_motor_rad_per_sec * (60.0 / (2.0 * math.pi))
        right_motor_rpm = right_motor_rad_per_sec * (60.0 / (2.0 * math.pi))
                
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left_motor_rpm
        right_msg.data = right_motor_rpm
        
        self.left_motor_rpm_pub_.publish(left_msg)
        self.right_motor_rpm_pub_.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
