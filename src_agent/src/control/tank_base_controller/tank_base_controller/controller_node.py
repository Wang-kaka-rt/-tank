#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorsState
from tank_base_controller.kinematics import MecanumKinematics

class TankBaseController(Node):
    def __init__(self):
        super().__init__('tank_base_controller')
        
        # Parameters
        self.declare_parameter('wheelbase', 0.1368)
        self.declare_parameter('track_width', 0.1446)
        self.declare_parameter('wheel_diameter', 0.065)
        
        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        wheel_diameter = self.get_parameter('wheel_diameter').value
        
        # Kinematics solver
        self.kinematics = MecanumKinematics(wheelbase, track_width, wheel_diameter)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.motor_pub = self.create_publisher(
            MotorsState,
            'ros_robot_controller/set_motor',
            10
        )
        
        self.get_logger().info('Tank Base Controller Started')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        # For tank drive, we ignore linear_y (lateral movement)
        linear_y = 0.0 
        angular_z = msg.angular.z
        
        # Calculate motor speeds
        motor_msg = self.kinematics.calculate_motor_speeds(linear_x, linear_y, angular_z)
        
        # Publish to hardware driver
        self.motor_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TankBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors on shutdown
        stop_msg = MotorsState()
        # Create empty motor states (which usually means stop or hold depending on driver implementation)
        # But better to send explicit 0 speed
        # Re-using kinematics to generate 0 speed message
        stop_msg = node.kinematics.calculate_motor_speeds(0.0, 0.0, 0.0)
        node.motor_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
