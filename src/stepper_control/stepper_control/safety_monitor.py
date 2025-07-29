#!/usr/bin/env python3
"""
Safety Monitor Module for Stepper Motor Control
Compatible with ROS2 Dashing and Ubuntu 18.04
Provides safety monitoring and emergency stop functionality
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
import threading
import time
import logging
from typing import Dict, Any, Optional

# Configure logging for Python 3.6 compatibility
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SafetyMonitor(Node):
    """
    Safety monitor for stepper motor control system
    Monitors system state and provides safety features
    """
    
    def __init__(self):
        """Initialize the safety monitor"""
        super().__init__('safety_monitor')
        
        # Safety parameters
        self.declare_parameter('max_azimuth_position', 1000)
        self.declare_parameter('min_azimuth_position', -1000)
        self.declare_parameter('max_elevator_position', 500)
        self.declare_parameter('min_elevator_position', -500)
        self.declare_parameter('max_velocity', 100.0)  # steps per second
        self.declare_parameter('safety_check_rate', 10.0)  # Hz
        
        # Safety limits
        self.max_azimuth = self.get_parameter('max_azimuth_position').value
        self.min_azimuth = self.get_parameter('min_azimuth_position').value
        self.max_elevator = self.get_parameter('max_elevator_position').value
        self.min_elevator = self.get_parameter('min_elevator_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_check_rate = self.get_parameter('safety_check_rate').value
        
        # Safety state
        self.emergency_stop_active = False
        self.safety_violation = False
        self.last_positions = {'azimuth': 0, 'elevator': 0}
        self.last_position_time = time.time()
        
        # Setup ROS2 interface
        self._setup_ros_interface()
        
        # Start safety monitoring thread
        self.running = True
        self.safety_thread = threading.Thread(target=self._safety_monitor_loop, daemon=True)
        self.safety_thread.start()
        
        logger.info("Safety monitor initialized")
    
    def _setup_ros_interface(self):
        """Setup ROS2 publishers and subscribers"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Publishers
        self.safety_status_pub = self.create_publisher(
            String,
            'safety/status',
            qos
        )
        
        self.emergency_stop_pub = self.create_publisher(
            String,
            'emergency_stop',
            qos
        )
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState,
            'stepper/joint_states',
            self._joint_states_callback,
            qos
        )
        
        self.stepper_status_sub = self.create_subscription(
            String,
            'stepper/status',
            self._stepper_status_callback,
            qos
        )
        
        logger.info("Safety monitor ROS2 interface setup completed")
    
    def _joint_states_callback(self, msg: JointState):
        """
        Callback for joint state messages
        
        Args:
            msg: JointState message containing position information
        """
        try:
            if len(msg.name) >= 2 and len(msg.position) >= 2:
                azimuth_pos = msg.position[0]
                elevator_pos = msg.position[1]
                
                # Check position limits
                if not self._check_position_limits(azimuth_pos, elevator_pos):
                    self._trigger_safety_violation("Position limits exceeded")
                
                # Check velocity limits
                current_time = time.time()
                if current_time - self.last_position_time > 0:
                    azimuth_velocity = abs(azimuth_pos - self.last_positions['azimuth']) / (current_time - self.last_position_time)
                    elevator_velocity = abs(elevator_pos - self.last_positions['elevator']) / (current_time - self.last_position_time)
                    
                    if azimuth_velocity > self.max_velocity or elevator_velocity > self.max_velocity:
                        self._trigger_safety_violation("Velocity limit exceeded")
                
                # Update last positions
                self.last_positions = {'azimuth': azimuth_pos, 'elevator': elevator_pos}
                self.last_position_time = current_time
                
        except Exception as e:
            self.get_logger().error("Error in joint states callback: %s", str(e))
    
    def _stepper_status_callback(self, msg: String):
        """
        Callback for stepper status messages
        
        Args:
            msg: String message containing stepper status
        """
        if "EMERGENCY_STOP" in msg.data:
            self.emergency_stop_active = True
            self.get_logger().warn("Emergency stop detected from stepper node")
    
    def _check_position_limits(self, azimuth_pos: float, elevator_pos: float) -> bool:
        """
        Check if positions are within safety limits
        
        Args:
            azimuth_pos: Current azimuth position
            elevator_pos: Current elevator position
            
        Returns:
            bool: True if positions are safe, False otherwise
        """
        if azimuth_pos > self.max_azimuth or azimuth_pos < self.min_azimuth:
            self.get_logger().warn("Azimuth position out of bounds: %f", azimuth_pos)
            return False
        
        if elevator_pos > self.max_elevator or elevator_pos < self.min_elevator:
            self.get_logger().warn("Elevator position out of bounds: %f", elevator_pos)
            return False
        
        return True
    
    def _trigger_safety_violation(self, reason: str):
        """
        Trigger a safety violation
        
        Args:
            reason: Reason for the safety violation
        """
        if not self.safety_violation:
            self.safety_violation = True
            self.emergency_stop_active = True
            
            # Publish emergency stop
            emergency_msg = String()
            emergency_msg.data = f"SAFETY_VIOLATION: {reason}"
            self.emergency_stop_pub.publish(emergency_msg)
            
            self.get_logger().error("Safety violation triggered: %s", reason)
    
    def _safety_monitor_loop(self):
        """Main safety monitoring loop"""
        rate = self.create_rate(self.safety_check_rate)
        
        while self.running:
            try:
                # Publish safety status
                status_msg = String()
                if self.emergency_stop_active:
                    status_msg.data = "EMERGENCY_STOP"
                elif self.safety_violation:
                    status_msg.data = "SAFETY_VIOLATION"
                else:
                    status_msg.data = "SAFE"
                
                self.safety_status_pub.publish(status_msg)
                
                # Check for stuck conditions (no position updates for extended time)
                current_time = time.time()
                if current_time - self.last_position_time > 5.0:  # 5 seconds timeout
                    self.get_logger().warn("No position updates received for 5 seconds")
                
                rate.sleep()
                
            except Exception as e:
                self.get_logger().error("Error in safety monitor loop: %s", str(e))
                time.sleep(0.1)
    
    def reset_safety_state(self):
        """Reset safety state (call after resolving safety issues)"""
        self.safety_violation = False
        self.emergency_stop_active = False
        self.get_logger().info("Safety state reset")
    
    def get_safety_status(self) -> Dict[str, Any]:
        """
        Get current safety status
        
        Returns:
            Dict containing safety status information
        """
        return {
            'emergency_stop_active': self.emergency_stop_active,
            'safety_violation': self.safety_violation,
            'last_positions': self.last_positions,
            'position_limits': {
                'azimuth': {'min': self.min_azimuth, 'max': self.max_azimuth},
                'elevator': {'min': self.min_elevator, 'max': self.max_elevator}
            },
            'velocity_limit': self.max_velocity
        }


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = SafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 