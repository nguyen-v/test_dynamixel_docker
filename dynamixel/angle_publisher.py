#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
from dynamixel_sdk import PortHandler, PacketHandler

class DynamixelAngleSubscriber(Node):
    def __init__(self):
        super().__init__('dynamixel_angle_subscriber')
        
        # Dynamixel configuration
        self.DEVICE_NAME = "/dev/sensors/dynamixel"  # Using the symlink from docker_run.sh
        self.BAUDRATE = 1000000
        self.DXL_ID = 1
        self.PROTOCOL_VERSION = 2.0
        
        # Control table addresses
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.TORQUE_ENABLE = 1
        
        # Position limits (0-4095 for full rotation)
        self.MIN_POSITION = 0
        self.MAX_POSITION = 4095
        self.POSITION_RANGE = self.MAX_POSITION - self.MIN_POSITION
        
        # Initialize Dynamixel communication
        self.port_handler = PortHandler(self.DEVICE_NAME)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        
        # Open port and set baudrate
        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open port")
            return
            
        if not self.port_handler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            return
            
        # Enable torque
        self.packet_handler.write1ByteTxRx(
            self.port_handler, 
            self.DXL_ID, 
            self.ADDR_TORQUE_ENABLE, 
            self.TORQUE_ENABLE
        )
        
        # Create subscriber for the angle topic
        self.subscription = self.create_subscription(
            Float64,
            '/dynamixel/angle',
            self.angle_callback,
            10)
        
        self.get_logger().info('Dynamixel Angle Subscriber Node has been started')
    
    def angle_callback(self, msg):
        """Callback function for received angle messages"""
        angle_degrees = msg.data
        self.get_logger().info(f'Received angle: {angle_degrees:.2f} degrees')
        
        # Convert angle to Dynamixel position (0-4095)
        position = int((angle_degrees / 360.0) * self.POSITION_RANGE)
        position = max(self.MIN_POSITION, min(self.MAX_POSITION, position))
        
        # Write position to motor
        self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self.DXL_ID,
            self.ADDR_GOAL_POSITION,
            position
        )
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'port_handler'):
            self.port_handler.closePort()

def main(args=None):
    rclpy.init(args=args)
    
    node = DynamixelAngleSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 