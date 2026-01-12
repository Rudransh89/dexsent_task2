#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class DualArmCoordinator(Node):
    def __init__(self):
        super().__init__('dual_arm_coordinator')
        
        # Publishers for Position Controllers
        self.left_pub = self.create_publisher(Float64MultiArray, '/left_arm_controller/commands', 10)
        self.right_pub = self.create_publisher(Float64MultiArray, '/right_arm_controller/commands', 10)
        
        self.get_logger().info("Coordinator Initialized. Starting sequence...")
        self.timer = self.create_timer(1.0, self.run_sequence)
        self.step = 0

    def run_sequence(self):
        msg = Float64MultiArray()
        
        if self.step < 3:
            # Phase 1: Move Left Arm to 90 degrees
            self.get_logger().info(f"Step {self.step}: Moving Left Arm to 90 degrees")
            msg.data = [1.57]
            self.left_pub.publish(msg)
            
        elif self.step < 6:
            # Phase 2: Move Right Arm to 90 degrees
            self.get_logger().info(f"Step {self.step}: Moving Right Arm to 90 degrees")
            msg.data = [1.57]
            self.right_pub.publish(msg)
            
            # Reset Left Arm to 0
            reset_msg = Float64MultiArray()
            reset_msg.data = [0.0]
            self.left_pub.publish(reset_msg)

        else:
            # Phase 3: Stop/Reset Everything
            self.get_logger().info("Sequence Complete. Resetting both arms.")
            msg.data = [0.0]
            self.left_pub.publish(msg)
            self.right_pub.publish(msg)
            self.destroy_node()
            exit()

        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = DualArmCoordinator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
