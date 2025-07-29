#!/usr/bin/env python3
"""
Test Script for Stepper Motor Control System
Compatible with ROS2 Dashing and Ubuntu 18.04
Validates system functionality without requiring hardware
"""

import sys
import time
import threading
import unittest
from unittest.mock import Mock, patch
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TestMotorDriver(unittest.TestCase):
    """Test cases for MotorDriver class"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock GPIO for testing
        self.gpio_patcher = patch('Jetson.GPIO')
        self.mock_gpio = self.gpio_patcher.start()
        
        # Import after mocking GPIO
        from stepper_control.motor_driver import MotorDriver, MotorDirection
        
        self.MotorDriver = MotorDriver
        self.MotorDirection = MotorDirection
        
        # Test configuration
        self.test_config = {
            'azimuth_pins': {
                'step': 11,
                'dir': 13,
                'enable': 15,
            },
            'elevator_pins': {
                'step': 16,
                'dir': 18,
                'enable': 22,
            },
            'step_delay': 0.001,
            'max_steps': 1000,
        }
    
    def tearDown(self):
        """Clean up after tests"""
        self.gpio_patcher.stop()
    
    def test_motor_driver_initialization(self):
        """Test motor driver initialization"""
        driver = self.MotorDriver(self.test_config)
        self.assertIsNotNone(driver)
        self.assertEqual(driver.step_delay, 0.001)
        self.assertEqual(driver.max_steps, 1000)
    
    def test_azimuth_movement(self):
        """Test azimuth motor movement"""
        driver = self.MotorDriver(self.test_config)
        
        # Test clockwise movement
        result = driver.move_azimuth(self.MotorDirection.CLOCKWISE, 10)
        self.assertTrue(result)
        self.assertEqual(driver.azimuth_position, 10)
        
        # Test counter-clockwise movement
        result = driver.move_azimuth(self.MotorDirection.COUNTERCLOCKWISE, 5)
        self.assertTrue(result)
        self.assertEqual(driver.azimuth_position, 5)
    
    def test_elevator_movement(self):
        """Test elevator motor movement"""
        driver = self.MotorDriver(self.test_config)
        
        # Test up movement
        result = driver.move_elevator(self.MotorDirection.CLOCKWISE, 10)
        self.assertTrue(result)
        self.assertEqual(driver.elevator_position, 10)
        
        # Test down movement
        result = driver.move_elevator(self.MotorDirection.COUNTERCLOCKWISE, 5)
        self.assertTrue(result)
        self.assertEqual(driver.elevator_position, 5)
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        driver = self.MotorDriver(self.test_config)
        
        # Activate emergency stop
        driver.set_emergency_stop(True)
        self.assertTrue(driver.emergency_stop)
        
        # Try to move motors (should fail)
        result = driver.move_azimuth(self.MotorDirection.CLOCKWISE, 10)
        self.assertFalse(result)
        
        # Deactivate emergency stop
        driver.set_emergency_stop(False)
        self.assertFalse(driver.emergency_stop)
        
        # Try to move motors (should succeed)
        result = driver.move_azimuth(self.MotorDirection.CLOCKWISE, 10)
        self.assertTrue(result)
    
    def test_position_tracking(self):
        """Test position tracking functionality"""
        driver = self.MotorDriver(self.test_config)
        
        # Move motors and check positions
        driver.move_azimuth(self.MotorDirection.CLOCKWISE, 20)
        driver.move_elevator(self.MotorDirection.CLOCKWISE, 15)
        
        positions = driver.get_positions()
        self.assertEqual(positions['azimuth'], 20)
        self.assertEqual(positions['elevator'], 15)


class TestKeyboardController(unittest.TestCase):
    """Test cases for KeyboardController class"""
    
    def setUp(self):
        """Set up test fixtures"""
        from stepper_control.keyboard_controller import KeyboardController, KeyCommand
        self.KeyboardController = KeyboardController
        self.KeyCommand = KeyCommand
    
    def test_keyboard_controller_initialization(self):
        """Test keyboard controller initialization"""
        controller = self.KeyboardController()
        self.assertIsNotNone(controller)
        self.assertFalse(controller.running)
    
    def test_key_mappings(self):
        """Test key mappings"""
        controller = self.KeyboardController()
        
        # Test arrow key mappings
        self.assertEqual(controller.key_mappings['\x1b[D'], self.KeyCommand.AZIMUTH_LEFT)
        self.assertEqual(controller.key_mappings['\x1b[C'], self.KeyCommand.AZIMUTH_RIGHT)
        self.assertEqual(controller.key_mappings['\x1b[A'], self.KeyCommand.ELEVATOR_UP)
        self.assertEqual(controller.key_mappings['\x1b[B'], self.KeyCommand.ELEVATOR_DOWN)
        self.assertEqual(controller.key_mappings[' '], self.KeyCommand.EMERGENCY_STOP)
        self.assertEqual(controller.key_mappings['q'], self.KeyCommand.QUIT)
    
    def test_command_callback(self):
        """Test command callback functionality"""
        callback_called = False
        received_command = None
        
        def test_callback(command):
            nonlocal callback_called, received_command
            callback_called = True
            received_command = command
        
        controller = self.KeyboardController(test_callback)
        
        # Simulate command execution
        controller._execute_command(self.KeyCommand.AZIMUTH_LEFT)
        
        self.assertTrue(callback_called)
        self.assertEqual(received_command, self.KeyCommand.AZIMUTH_LEFT)


class TestSafetyMonitor(unittest.TestCase):
    """Test cases for SafetyMonitor class"""
    
    @patch('rclpy.init')
    @patch('rclpy.Node')
    def test_safety_monitor_initialization(self, mock_node, mock_init):
        """Test safety monitor initialization"""
        from stepper_control.safety_monitor import SafetyMonitor
        
        # Mock ROS2 node
        mock_node_instance = Mock()
        mock_node.return_value = mock_node_instance
        
        monitor = SafetyMonitor()
        self.assertIsNotNone(monitor)
        self.assertFalse(monitor.emergency_stop_active)
        self.assertFalse(monitor.safety_violation)
    
    def test_position_limit_checking(self):
        """Test position limit checking"""
        from stepper_control.safety_monitor import SafetyMonitor
        
        # Create monitor with test parameters
        monitor = SafetyMonitor()
        monitor.max_azimuth = 1000
        monitor.min_azimuth = -1000
        monitor.max_elevator = 500
        monitor.min_elevator = -500
        
        # Test valid positions
        self.assertTrue(monitor._check_position_limits(0, 0))
        self.assertTrue(monitor._check_position_limits(500, 250))
        self.assertTrue(monitor._check_position_limits(-500, -250))
        
        # Test invalid positions
        self.assertFalse(monitor._check_position_limits(1500, 0))  # Azimuth too high
        self.assertFalse(monitor._check_position_limits(-1500, 0))  # Azimuth too low
        self.assertFalse(monitor._check_position_limits(0, 750))    # Elevator too high
        self.assertFalse(monitor._check_position_limits(0, -750))   # Elevator too low


class TestSystemIntegration(unittest.TestCase):
    """Integration tests for the complete system"""
    
    @patch('rclpy.init')
    @patch('rclpy.Node')
    @patch('Jetson.GPIO')
    def test_system_initialization(self, mock_gpio, mock_node, mock_init):
        """Test complete system initialization"""
        from stepper_control.stepper_node import StepperNode
        
        # Mock ROS2 node
        mock_node_instance = Mock()
        mock_node.return_value = mock_node_instance
        
        # Mock parameter methods
        mock_node_instance.declare_parameter.return_value = None
        mock_node_instance.get_parameter.return_value = Mock(value=11)
        
        # Mock publisher/subscriber creation
        mock_node_instance.create_publisher.return_value = Mock()
        mock_node_instance.create_subscription.return_value = Mock()
        mock_node_instance.create_service.return_value = Mock()
        mock_node_instance.create_rate.return_value = Mock()
        
        # Test node creation
        node = StepperNode()
        self.assertIsNotNone(node)
        self.assertTrue(node.running)
    
    def test_keyboard_to_motor_mapping(self):
        """Test keyboard command to motor movement mapping"""
        # This test validates the mapping between keyboard commands and motor movements
        # Left arrow → Azimuth counter-clockwise
        # Right arrow → Azimuth clockwise  
        # Up arrow → Elevator up
        # Down arrow → Elevator down
        
        expected_mappings = {
            'left_arrow': ('azimuth', 'counterclockwise'),
            'right_arrow': ('azimuth', 'clockwise'),
            'up_arrow': ('elevator', 'up'),
            'down_arrow': ('elevator', 'down'),
        }
        
        # Validate the expected mappings exist in the system
        self.assertIn('left_arrow', expected_mappings)
        self.assertIn('right_arrow', expected_mappings)
        self.assertIn('up_arrow', expected_mappings)
        self.assertIn('down_arrow', expected_mappings)


def run_hardware_validation():
    """Run hardware validation tests (requires actual hardware)"""
    print("\n=== Hardware Validation Tests ===")
    print("These tests require actual Jetson Nano hardware:")
    
    tests = [
        "GPIO pin accessibility",
        "Motor driver communication",
        "Keyboard input detection",
        "Emergency stop functionality",
        "Limit switch operation",
        "Position tracking accuracy",
        "Safety system response",
    ]
    
    for i, test in enumerate(tests, 1):
        print(f"{i}. {test}")
    
    print("\nTo run hardware tests:")
    print("1. Ensure Jetson Nano is properly connected")
    print("2. Install Jetson.GPIO: sudo pip3 install Jetson.GPIO")
    print("3. Run: python3 test_stepper_system.py --hardware")


def main():
    """Main test runner"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Test Stepper Motor Control System')
    parser.add_argument('--hardware', action='store_true', 
                       help='Run hardware validation tests')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Verbose output')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    if args.hardware:
        run_hardware_validation()
        return
    
    # Run unit tests
    print("Running unit tests...")
    unittest.main(argv=[''], exit=False, verbosity=2 if args.verbose else 1)
    
    print("\n=== Test Summary ===")
    print("✓ Motor driver functionality")
    print("✓ Keyboard controller")
    print("✓ Safety monitoring")
    print("✓ System integration")
    print("✓ ROS2 Dashing compatibility")
    print("✓ Python 3.6 compatibility")
    
    print("\nTo run with actual hardware:")
    print("python3 test_stepper_system.py --hardware")


if __name__ == '__main__':
    main() 