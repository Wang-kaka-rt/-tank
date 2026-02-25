import math
from ros_robot_controller_msgs.msg import MotorState, MotorsState

class MecanumKinematics:
    """
    Kinematics for Mecanum chassis.
    For tank/differential drive, set linear_y to 0.
    """
    def __init__(self, wheelbase=0.1368, track_width=0.1446, wheel_diameter=0.065):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_to_rps(self, speed):
        """
        Convert speed (m/s) to rotations per second (rps).
        """
        return speed / (math.pi * self.wheel_diameter)

    def calculate_motor_speeds(self, linear_x, linear_y, angular_z):
        """
        Calculate individual motor speeds based on robot velocity.
        
        Args:
            linear_x (float): Forward/backward speed (m/s)
            linear_y (float): Left/right speed (m/s). Set to 0 for tank mode.
            angular_z (float): Rotation speed (rad/s)
            
        Returns:
            MotorsState: ROS message containing motor speeds.
        """
        # Mecanum kinematics formula
        # motor1: front-left
        # motor2: rear-left
        # motor3: front-right
        # motor4: rear-right
        
        # Calculate linear velocity components for each wheel
        # Note: The signs depend on the motor mounting and wiring.
        # Assuming standard mecanum configuration:
        # v1 = vx - vy - (Lx + Ly)*w
        # v2 = vx + vy - (Lx + Ly)*w
        # v3 = vx + vy + (Lx + Ly)*w
        # v4 = vx - vy + (Lx + Ly)*w
        
        k = (self.wheelbase + self.track_width) / 2.0
        
        # Calculate raw linear speeds for each wheel
        v1 = linear_x - linear_y - angular_z * k
        v2 = linear_x + linear_y - angular_z * k
        v3 = linear_x + linear_y + angular_z * k
        v4 = linear_x - linear_y + angular_z * k
        
        # Convert to RPS and adjust signs based on original code reference
        # Based on mecanum.py:
        # v_s = [self.speed_covert(v) for v in [-motor1, -motor2, motor3, motor4]]
        # So motor 1 & 2 are inverted.
        
        rps1 = self.speed_to_rps(-v1)
        rps2 = self.speed_to_rps(-v2)
        rps3 = self.speed_to_rps(v3)
        rps4 = self.speed_to_rps(v4)
        
        speeds = [rps1, rps2, rps3, rps4]
        
        # Create message
        msg = MotorsState()
        data = []
        for i, speed in enumerate(speeds):
            motor_msg = MotorState()
            motor_msg.id = i + 1
            motor_msg.rps = float(speed)
            data.append(motor_msg)
            
        msg.data = data
        return msg
