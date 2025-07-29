#!/usr/bin/env python3
"""
Keyboard Controller Module for Stepper Motor Control
Compatible with ROS2 Dashing and Ubuntu 18.04
Provides real-time keyboard input handling for motor control
"""

import sys
import tty
import termios
import threading
import time
import logging
from typing import Optional, Callable, Dict, Any
from enum import Enum

# Configure logging for Python 3.6 compatibility
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class KeyCommand(Enum):
    """Keyboard command enumeration"""
    AZIMUTH_LEFT = "azimuth_left"      # Left arrow
    AZIMUTH_RIGHT = "azimuth_right"    # Right arrow
    ELEVATOR_UP = "elevator_up"        # Up arrow
    ELEVATOR_DOWN = "elevator_down"    # Down arrow
    EMERGENCY_STOP = "emergency_stop"  # Space/Enter
    QUIT = "quit"                      # 'q' key


class KeyboardController:
    """
    Real-time keyboard controller for stepper motor control
    Provides non-blocking keyboard input with arrow key mapping
    """
    
    def __init__(self, command_callback: Optional[Callable] = None):
        """
        Initialize keyboard controller
        
        Args:
            command_callback: Optional callback function for key commands
        """
        self.command_callback = command_callback
        self.running = False
        self.key_thread = None
        self.old_settings = None
        
        # Key mappings for arrow keys
        self.key_mappings = {
            '\x1b[D': KeyCommand.AZIMUTH_LEFT,      # Left arrow
            '\x1b[C': KeyCommand.AZIMUTH_RIGHT,     # Right arrow
            '\x1b[A': KeyCommand.ELEVATOR_UP,       # Up arrow
            '\x1b[B': KeyCommand.ELEVATOR_DOWN,     # Down arrow
            ' ': KeyCommand.EMERGENCY_STOP,         # Space
            '\r': KeyCommand.EMERGENCY_STOP,        # Enter
            '\n': KeyCommand.EMERGENCY_STOP,        # Enter (alternative)
            'q': KeyCommand.QUIT,                   # Quit
            'Q': KeyCommand.QUIT,                   # Quit (uppercase)
        }
        
        # Command repeat settings
        self.repeat_delay = 0.1  # 100ms delay between repeated commands
        self.last_command_time = 0
        
        logger.info("Keyboard controller initialized")
    
    def _get_char(self) -> str:
        """
        Get a single character from stdin without blocking
        
        Returns:
            str: The character read, or empty string if no input
        """
        try:
            # Set terminal to raw mode
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(sys.stdin.fileno())
            
            # Try to read a character
            ch = sys.stdin.read(1)
            
            # Check for arrow keys (3-character sequence)
            if ch == '\x1b':
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    ch3 = sys.stdin.read(1)
                    ch = ch + ch2 + ch3
            
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
            return ch
            
        except (termios.error, OSError):
            return ""
        except Exception as e:
            logger.error("Error reading keyboard input: %s", str(e))
            return ""
    
    def _keyboard_loop(self):
        """Main keyboard input loop"""
        logger.info("Starting keyboard input loop")
        logger.info("Controls: Arrow keys for movement, Space/Enter for emergency stop, 'q' to quit")
        
        while self.running:
            try:
                # Get character input
                char = self._get_char()
                
                if char:
                    # Check if it's a mapped key
                    if char in self.key_mappings:
                        command = self.key_mappings[char]
                        
                        # Check repeat delay
                        current_time = time.time()
                        if current_time - self.last_command_time >= self.repeat_delay:
                            self.last_command_time = current_time
                            
                            # Execute command
                            self._execute_command(command)
                            
                            # Log command
                            logger.debug("Executed command: %s", command.value)
                
                # Small delay to prevent high CPU usage
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received")
                break
            except Exception as e:
                logger.error("Error in keyboard loop: %s", str(e))
                break
        
        logger.info("Keyboard input loop stopped")
    
    def _execute_command(self, command: KeyCommand):
        """
        Execute a keyboard command
        
        Args:
            command: The command to execute
        """
        if command == KeyCommand.QUIT:
            logger.info("Quit command received")
            self.stop()
            return
        
        # Call the callback function if provided
        if self.command_callback:
            try:
                self.command_callback(command)
            except Exception as e:
                logger.error("Error in command callback: %s", str(e))
        else:
            # Default command handling
            logger.info("Command received: %s", command.value)
    
    def start(self):
        """Start the keyboard controller"""
        if self.running:
            logger.warning("Keyboard controller already running")
            return
        
        self.running = True
        self.key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.key_thread.start()
        logger.info("Keyboard controller started")
    
    def stop(self):
        """Stop the keyboard controller"""
        self.running = False
        if self.key_thread:
            self.key_thread.join(timeout=1.0)
        logger.info("Keyboard controller stopped")
    
    def set_command_callback(self, callback: Callable):
        """
        Set the command callback function
        
        Args:
            callback: Function to call when commands are received
        """
        self.command_callback = callback
        logger.info("Command callback set")


def main():
    """Test function for keyboard controller"""
    def test_callback(command):
        """Test callback function"""
        print(f"Command received: {command.value}")
    
    controller = KeyboardController(test_callback)
    
    try:
        print("Keyboard controller test")
        print("Use arrow keys to test, 'q' to quit")
        controller.start()
        
        # Keep running until quit
        while controller.running:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.stop()


if __name__ == '__main__':
    main() 