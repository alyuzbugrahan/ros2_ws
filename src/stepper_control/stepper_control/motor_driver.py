#!/usr/bin/env python3
"""
Motor Driver Module for Jetson Nano Stepper Control
Compatible with ROS2 Dashing and Ubuntu 18.04
Supports independent dual motor control with safety features
"""

import Jetson.GPIO as GPIO
import time
import threading
import logging
from typing import Optional, Dict, Any
from enum import Enum

# Configure logging for Python 3.6 compatibility
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MotorType(Enum):
    """Motor type enumeration"""
    AZIMUTH = "azimuth"
    ELEVATOR = "elevator"


class MotorDirection(Enum):
    """Motor direction enumeration"""
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"


class MotorDriver:
    """
    Dual stepper motor driver for Jetson Nano
    Supports independent control of azimuth and elevator motors
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize motor driver with configuration
        
        Args:
            config: Dictionary containing GPIO pin assignments and motor parameters
        """
        self.config = config
        self.azimuth_pins = config['azimuth_pins']
        self.elevator_pins = config['elevator_pins']
        self.step_delay = config.get('step_delay', 0.001)  # 1ms default
        self.max_steps = config.get('max_steps', 1000)
        
        # Motor state tracking
        self.azimuth_position = 0
        self.elevator_position = 0
        self.azimuth_target = 0
        self.elevator_target = 0
        
        # Safety flags
        self.emergency_stop = False
        self.limit_switches = {
            'azimuth_min': False,
            'azimuth_max': False,
            'elevator_min': False,
            'elevator_max': False
        }
        
        # Threading locks for thread safety
        self.azimuth_lock = threading.Lock()
        self.elevator_lock = threading.Lock()
        self.gpio_lock = threading.Lock()
        
        # Initialize GPIO
        self._setup_gpio()
        
        logger.info("Motor driver initialized successfully")
    
    def _setup_gpio(self):
        """Initialize GPIO pins for both motors"""
        try:
            # Set GPIO mode for Jetson Nano
            GPIO.setmode(GPIO.BOARD)
            
            # Configure azimuth motor pins
            for pin in self.azimuth_pins.values():
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
            
            # Configure elevator motor pins
            for pin in self.elevator_pins.values():
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
            
            # Configure limit switch pins if specified
            if 'limit_switches' in self.config:
                for switch_pin in self.config['limit_switches'].values():
                    GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            logger.info("GPIO setup completed")
            
        except Exception as e:
            logger.error("GPIO setup failed: %s", str(e))
            raise
    
    def _check_safety_limits(self, motor_type: MotorType, direction: MotorDirection) -> bool:
        """
        Check if movement is safe
        
        Args:
            motor_type: Type of motor (azimuth or elevator)
            direction: Direction of movement
            
        Returns:
            bool: True if movement is safe, False otherwise
        """
        if self.emergency_stop:
            logger.warning("Emergency stop active")
            return False
        
        # Check limit switches
        if motor_type == MotorType.AZIMUTH:
            if direction == MotorDirection.CLOCKWISE and self.limit_switches['azimuth_max']:
                logger.warning("Azimuth max limit reached")
                return False
            elif direction == MotorDirection.COUNTERCLOCKWISE and self.limit_switches['azimuth_min']:
                logger.warning("Azimuth min limit reached")
                return False
        elif motor_type == MotorType.ELEVATOR:
            if direction == MotorDirection.CLOCKWISE and self.limit_switches['elevator_max']:
                logger.warning("Elevator max limit reached")
                return False
            elif direction == MotorDirection.COUNTERCLOCKWISE and self.limit_switches['elevator_min']:
                logger.warning("Elevator min limit reached")
                return False
        
        return True
    
    def _update_limit_switches(self):
        """Update limit switch states"""
        try:
            if 'limit_switches' in self.config:
                switches = self.config['limit_switches']
                self.limit_switches['azimuth_min'] = GPIO.input(switches.get('azimuth_min', 0)) == GPIO.LOW
                self.limit_switches['azimuth_max'] = GPIO.input(switches.get('azimuth_max', 0)) == GPIO.LOW
                self.limit_switches['elevator_min'] = GPIO.input(switches.get('elevator_min', 0)) == GPIO.LOW
                self.limit_switches['elevator_max'] = GPIO.input(switches.get('elevator_max', 0)) == GPIO.LOW
        except Exception as e:
            logger.error("Error reading limit switches: %s", str(e))
    
    def _step_motor(self, motor_type: MotorType, direction: MotorDirection, steps: int = 1):
        """
        Execute motor steps
        
        Args:
            motor_type: Type of motor to control
            direction: Direction of movement
            steps: Number of steps to execute
        """
        if not self._check_safety_limits(motor_type, direction):
            return False
        
        pins = self.azimuth_pins if motor_type == MotorType.AZIMUTH else self.elevator_pins
        lock = self.azimuth_lock if motor_type == MotorType.AZIMUTH else self.elevator_lock
        
        with lock:
            try:
                for _ in range(steps):
                    if not self._check_safety_limits(motor_type, direction):
                        break
                    
                    # Generate step pulse
                    with self.gpio_lock:
                        GPIO.output(pins['step'], GPIO.HIGH)
                        time.sleep(self.step_delay)
                        GPIO.output(pins['step'], GPIO.LOW)
                        time.sleep(self.step_delay)
                    
                    # Update position
                    if motor_type == MotorType.AZIMUTH:
                        if direction == MotorDirection.CLOCKWISE:
                            self.azimuth_position += 1
                        else:
                            self.azimuth_position -= 1
                    else:
                        if direction == MotorDirection.CLOCKWISE:
                            self.elevator_position += 1
                        else:
                            self.elevator_position -= 1
                
                return True
                
            except Exception as e:
                logger.error("Error during motor stepping: %s", str(e))
                return False
    
    def move_azimuth(self, direction: MotorDirection, steps: int = 1) -> bool:
        """
        Move azimuth motor
        
        Args:
            direction: Direction of movement
            steps: Number of steps to move
            
        Returns:
            bool: True if movement successful, False otherwise
        """
        return self._step_motor(MotorType.AZIMUTH, direction, steps)
    
    def move_elevator(self, direction: MotorDirection, steps: int = 1) -> bool:
        """
        Move elevator motor
        
        Args:
            direction: Direction of movement
            steps: Number of steps to move
            
        Returns:
            bool: True if movement successful, False otherwise
        """
        return self._step_motor(MotorType.ELEVATOR, direction, steps)
    
    def get_positions(self) -> Dict[str, int]:
        """
        Get current motor positions
        
        Returns:
            Dict containing azimuth and elevator positions
        """
        return {
            'azimuth': self.azimuth_position,
            'elevator': self.elevator_position
        }
    
    def set_emergency_stop(self, stop: bool = True):
        """
        Set emergency stop state
        
        Args:
            stop: True to activate emergency stop, False to deactivate
        """
        self.emergency_stop = stop
        if stop:
            logger.warning("Emergency stop activated")
            # Stop all motors by setting enable pins low
            with self.gpio_lock:
                GPIO.output(self.azimuth_pins['enable'], GPIO.HIGH)
                GPIO.output(self.elevator_pins['enable'], GPIO.HIGH)
        else:
            logger.info("Emergency stop deactivated")
            # Re-enable motors
            with self.gpio_lock:
                GPIO.output(self.azimuth_pins['enable'], GPIO.LOW)
                GPIO.output(self.elevator_pins['enable'], GPIO.LOW)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        try:
            with self.gpio_lock:
                GPIO.cleanup()
            logger.info("GPIO cleanup completed")
        except Exception as e:
            logger.error("Error during GPIO cleanup: %s", str(e))


def main():
    """Test function for motor driver"""
    # Example configuration
    config = {
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
    }
    
    try:
        driver = MotorDriver(config)
        
        # Test movements
        print("Testing azimuth motor...")
        driver.move_azimuth(MotorDirection.CLOCKWISE, 10)
        time.sleep(1)
        driver.move_azimuth(MotorDirection.COUNTERCLOCKWISE, 10)
        
        print("Testing elevator motor...")
        driver.move_elevator(MotorDirection.CLOCKWISE, 10)
        time.sleep(1)
        driver.move_elevator(MotorDirection.COUNTERCLOCKWISE, 10)
        
        print("Current positions:", driver.get_positions())
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'driver' in locals():
            driver.cleanup()


if __name__ == '__main__':
    main() 