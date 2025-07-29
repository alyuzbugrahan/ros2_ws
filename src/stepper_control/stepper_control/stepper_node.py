#!/usr/bin/env python3
"""
Main Stepper Control Node for ROS2 Dashing
Compatible with Ubuntu 18.04 and Jetson Nano
Integrates motor driver and keyboard controller
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import threading
import time
import logging
from typing import Dict, Any

from .motor_driver import MotorDriver, MotorDirection
from .keyboard_controller import KeyboardController, KeyCommand

# Configure logging for Python 3.6 compatibility
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class StepperNode(Node):
    """
    Main ROS2 node for stepper motor control
    Integrates motor driver and keyboard controller
    """
    
    def __init__(self):
        """Initialize the stepper control node"""
        super().__init__('stepper_control_node')
        
        # Load configuration
        self.config = self._load_config()
        
        # Initialize motor driver
        self.motor_driver = MotorDriver(self.config['motor'])
        
        # Initialize keyboard controller
        self.keyboard_controller = KeyboardController(self._keyboard_callback)
        
        # Node state
        self.running = True
        self.emergency_stop_active = False
        
        # Setup ROS2 publishers and subscribers
        self._setup_ros_interface()
        
        # Start keyboard controller
        self.keyboard_controller.start()
        
        # Start status publishing thread
        self.status_thread = threading.Thread(target=self._status_publisher_loop, daemon=True)
        self.status_thread.start()
        
        logger.info("Stepper control node initialized")
    
    def _load_config(self) -> Dict[str, Any]:
        """
        Load configuration from parameters
        
        Returns:
            Dict containing motor and control configuration
        """
        # Default configuration
        config = {
            'motor': {
                'azimuth_pins': {
                    'step': 11,      # GPIO17
                    'dir': 13,        # GPIO27
                    'enable': 15,     # GPIO22
                },
                'elevator_pins': {
                    'step': 16,       # GPIO23
                    'dir': 18,        # GPIO24
                    'enable': 22,     # GPIO25
                },
                'step_delay': 0.001,
                'max_steps': 1000,
                'limit_switches': {
                    'azimuth_min': 29,  # GPIO5
                    'azimuth_max': 31,  # GPIO6
                    'elevator_min': 33, # GPIO13
                    'elevator_max': 35, # GPIO19
                }
            },
            'control': {
                'steps_per_command': 10,
                'status_publish_rate': 10.0,  # Hz
            }
        }
        
        # Override with ROS2 parameters if available
        self.declare_parameter('azimuth_step_pin', 11)
        self.declare_parameter('azimuth_dir_pin', 13)
        self.declare_parameter('azimuth_enable_pin', 15)
        self.declare_parameter('elevator_step_pin', 16)
        self.declare_parameter('elevator_dir_pin', 18)
        self.declare_parameter('elevator_enable_pin', 22)
        self.declare_parameter('step_delay', 0.001)
        self.declare_parameter('steps_per_command', 10)
        
        # Update config with parameters
        config['motor']['azimuth_pins']['step'] = self.get_parameter('azimuth_step_pin').value
        config['motor']['azimuth_pins']['dir'] = self.get_parameter('azimuth_dir_pin').value
        config['motor']['azimuth_pins']['enable'] = self.get_parameter('azimuth_enable_pin').value
        config['motor']['elevator_pins']['step'] = self.get_parameter('elevator_step_pin').value
        config['motor']['elevator_pins']['dir'] = self.get_parameter('elevator_dir_pin').value
        config['motor']['elevator_pins']['enable'] = self.get_parameter('elevator_enable_pin').value
        config['motor']['step_delay'] = self.get_parameter('step_delay').value
        config['control']['steps_per_command'] = self.get_parameter('steps_per_command').value
        
        return config
    
    def _setup_ros_interface(self):
        """Setup ROS2 publishers and subscribers"""
        # QoS profile for reliable communication
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, 
            'stepper/status', 
            qos
        )
        
        self.position_pub = self.create_publisher(
            JointState, 
            'stepper/joint_states', 
            qos
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            qos
        )
        
        self.emergency_stop_sub = self.create_subscription(
            String,
            'emergency_stop',
            self._emergency_stop_callback,
            qos
        )
        
        # Services
        self.get_position_srv = self.create_service(
            'stepper/get_position',
            'std_srvs/Trigger',
            self._get_position_service
        )
        
        logger.info("ROS2 interface setup completed")
    
    def _keyboard_callback(self, command: KeyCommand):
        """
        Callback for keyboard commands
        
        Args:
            command: The keyboard command received
        """
        if not self.running:
            return
        
        try:
            if command == KeyCommand.AZIMUTH_LEFT:
                self.motor_driver.move_azimuth(
                    MotorDirection.COUNTERCLOCKWISE, 
                    self.config['control']['steps_per_command']
                )
                self.get_logger().info("Azimuth left command executed")
                
            elif command == KeyCommand.AZIMUTH_RIGHT:
                self.motor_driver.move_azimuth(
                    MotorDirection.CLOCKWISE, 
                    self.config['control']['steps_per_command']
                )
                self.get_logger().info("Azimuth right command executed")
                
            elif command == KeyCommand.ELEVATOR_UP:
                self.motor_driver.move_elevator(
                    MotorDirection.CLOCKWISE, 
                    self.config['control']['steps_per_command']
                )
                self.get_logger().info("Elevator up command executed")
                
            elif command == KeyCommand.ELEVATOR_DOWN:
                self.motor_driver.move_elevator(
                    MotorDirection.COUNTERCLOCKWISE, 
                    self.config['control']['steps_per_command']
                )
                self.get_logger().info("Elevator down command executed")
                
            elif command == KeyCommand.EMERGENCY_STOP:
                self._activate_emergency_stop()
                
        except Exception as e:
            self.get_logger().error("Error executing keyboard command: %s", str(e))
    
    def _cmd_vel_callback(self, msg: Twist):
        """
        Callback for velocity commands
        
        Args:
            msg: Twist message containing velocity commands
        """
        if not self.running or self.emergency_stop_active:
            return
        
        try:
            # Convert velocity to motor steps
            # This is a simplified conversion - adjust based on your motor specs
            steps_per_radian = 100  # Adjust based on your motor
            
            # Azimuth control (angular.z)
            if abs(msg.angular.z) > 0.01:
                direction = MotorDirection.CLOCKWISE if msg.angular.z > 0 else MotorDirection.COUNTERCLOCKWISE
                steps = int(abs(msg.angular.z) * steps_per_radian)
                self.motor_driver.move_azimuth(direction, steps)
            
            # Elevator control (linear.y)
            if abs(msg.linear.y) > 0.01:
                direction = MotorDirection.CLOCKWISE if msg.linear.y > 0 else MotorDirection.COUNTERCLOCKWISE
                steps = int(abs(msg.linear.y) * steps_per_radian)
                self.motor_driver.move_elevator(direction, steps)
                
        except Exception as e:
            self.get_logger().error("Error executing velocity command: %s", str(e))
    
    def _emergency_stop_callback(self, msg: String):
        """
        Callback for emergency stop commands
        
        Args:
            msg: String message (any non-empty string activates emergency stop)
        """
        if msg.data:
            self._activate_emergency_stop()
    
    def _activate_emergency_stop(self):
        """Activate emergency stop"""
        self.emergency_stop_active = True
        self.motor_driver.set_emergency_stop(True)
        self.get_logger().warn("Emergency stop activated")
        
        # Publish emergency stop status
        status_msg = String()
        status_msg.data = "EMERGENCY_STOP"
        self.status_pub.publish(status_msg)
    
    def _get_position_service(self, request, response):
        """
        Service callback for getting current position
        
        Args:
            request: Service request
            response: Service response
            
        Returns:
            Service response with position data
        """
        try:
            positions = self.motor_driver.get_positions()
            response.success = True
            response.message = f"Azimuth: {positions['azimuth']}, Elevator: {positions['elevator']}"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error getting position: {str(e)}"
            return response
    
    def _status_publisher_loop(self):
        """Publish status and position information"""
        rate = self.create_rate(self.config['control']['status_publish_rate'])
        
        while self.running:
            try:
                # Publish joint states
                joint_msg = JointState()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.name = ['azimuth_joint', 'elevator_joint']
                
                positions = self.motor_driver.get_positions()
                joint_msg.position = [positions['azimuth'], positions['elevator']]
                joint_msg.velocity = [0.0, 0.0]  # Not implemented yet
                joint_msg.effort = [0.0, 0.0]    # Not implemented yet
                
                self.position_pub.publish(joint_msg)
                
                # Publish status
                status_msg = String()
                if self.emergency_stop_active:
                    status_msg.data = "EMERGENCY_STOP"
                else:
                    status_msg.data = f"RUNNING - Azimuth: {positions['azimuth']}, Elevator: {positions['elevator']}"
                
                self.status_pub.publish(status_msg)
                
                rate.sleep()
                
            except Exception as e:
                self.get_logger().error("Error in status publisher: %s", str(e))
                time.sleep(0.1)
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        self.keyboard_controller.stop()
        self.motor_driver.cleanup()
        self.get_logger().info("Stepper node cleanup completed")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = StepperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 