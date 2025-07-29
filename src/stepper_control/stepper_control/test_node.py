#!/usr/bin/env python3
"""
Simple test node for debugging build issues
"""

import rclpy
from rclpy.node import Node

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = Node('test_node')
        node.get_logger().info('Test node is running successfully!')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 