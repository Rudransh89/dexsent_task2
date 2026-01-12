#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class CustomCartesianController(Node):
    def __init__(self):
        super().__init__('custom_cartesian_controller')
        
        # 1. Output: Publish to robot joints
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 2. Input: Subscribe to Cartesian Commands
        # Format: [Left_X, Left_Y, Left_Z, Right_X, Right_Y, Right_Z]
        self.sub = self.create_subscription(Float64MultiArray, '/dual_arm/command', self.command_callback, 10)
        
        # Robot Link Lengths (Approximating Fanuc CRX)
        self.L1 = 0.7 
        self.L2 = 0.5 
        
        self.get_logger().info("Custom Cartesian Controller Ready. Waiting for commands...")

    def solve_ik(self, x, y, z):
        """ Inverse Kinematics: Converts XYZ -> Joint Angles """
        waist = math.atan2(y, x)
        d = math.sqrt(x*x + y*y)
        z_eff = z - 0.3 
        r = math.sqrt(d*d + z_eff*z_eff)
        
        try:
            alpha = math.acos((self.L1**2 + r**2 - self.L2**2) / (2 * self.L1 * r))
            beta = math.atan2(z_eff, d)
            gamma = math.acos((self.L1**2 + self.L2**2 - r**2) / (2 * self.L1 * self.L2))
            
            shoulder = (math.pi / 2) - (alpha + beta)
            elbow = math.pi - gamma
            
            # Return 6 Joints: [J1, J2, J3, J4, J5, J6]
            return [waist, -shoulder, -elbow, 0.0, -1.0, 0.0]
        except ValueError:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def command_callback(self, msg):
        # Unpack Command
        lx, ly, lz = msg.data[0], msg.data[1], msg.data[2]
        rx, ry, rz = msg.data[3], msg.data[4], msg.data[5]

        # Calculate Joints
        l_joints = self.solve_ik(lx, ly, lz)
        r_joints = self.solve_ik(rx, ry, rz)

        # Publish
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4', 'left_joint_5', 'left_joint_6',
            'right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6'
        ]
        joint_msg.position = l_joints + r_joints
        self.joint_pub.publish(joint_msg)

def main():
    rclpy.init()
    node = CustomCartesianController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
