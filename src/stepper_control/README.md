# Jetson Nano Stepper Motor Control System

A comprehensive ROS2 Dashing package for controlling dual stepper motors on Jetson Nano with real-time keyboard interface and safety monitoring.

## 🎯 Features

- **Dual Motor Control**: Independent control of azimuth (horizontal) and elevator (vertical) stepper motors
- **Real-time Keyboard Interface**: Arrow key control with non-blocking input handling
- **Safety Monitoring**: Position limits, velocity limits, and emergency stop functionality
- **ROS2 Dashing Compatible**: Designed specifically for Ubuntu 18.04 and ROS2 Dashing
- **Jetson Nano Optimized**: Uses Jetson.GPIO library for hardware control
- **Thread-safe Design**: Proper threading for concurrent motor and keyboard control

## 🏗️ Hardware Requirements

### Jetson Nano Setup
- **Platform**: Jetson Nano JNX30D
- **OS**: Ubuntu 18.04 LTS
- **Architecture**: ARM64 (aarch64)

### Motor Hardware
- **Motors**: 2x Stepper Motors (Azimuth + Elevator)
- **Drivers**: 2x Stepper Motor Drivers (one per motor)
- **Connections**: GPIO-based control interface

## 🔧 Hardware Connections

### GPIO Pin Assignments (BOARD numbering)

#### Azimuth Motor (Horizontal)
| Signal | GPIO Pin | Board Pin | Description |
|--------|----------|-----------|-------------|
| STEP   | GPIO17   | 11        | Step pulse signal |
| DIR    | GPIO27   | 13        | Direction control |
| ENABLE | GPIO22   | 15        | Motor enable/disable |

#### Elevator Motor (Vertical)
| Signal | GPIO Pin | Board Pin | Description |
|--------|----------|-----------|-------------|
| STEP   | GPIO23   | 16        | Step pulse signal |
| DIR    | GPIO24   | 18        | Direction control |
| ENABLE | GPIO25   | 22        | Motor enable/disable |

#### Limit Switches (Optional)
| Switch | GPIO Pin | Board Pin | Description |
|--------|----------|-----------|-------------|
| Azimuth Min | GPIO5  | 29        | Azimuth minimum limit |
| Azimuth Max | GPIO6  | 31        | Azimuth maximum limit |
| Elevator Min | GPIO13 | 33        | Elevator minimum limit |
| Elevator Max | GPIO19 | 35        | Elevator maximum limit |

### Wiring Diagram
```
Jetson Nano GPIO Header
┌─────────────────┐
│ 3.3V  │ 5V     │
│ GPIO17 │ 5V     │ ← Azimuth STEP (Pin 11)
│ GPIO27 │ GND    │ ← Azimuth DIR (Pin 13)
│ GPIO22 │ GPIO23 │ ← Azimuth ENABLE (Pin 15)
│ 3.3V  │ GPIO24 │ ← Elevator STEP (Pin 16)
│ GPIO27 │ GPIO25 │ ← Elevator DIR (Pin 18)
│ GPIO22 │ GPIO5  │ ← Elevator ENABLE (Pin 22)
│ 3.3V  │ GPIO6  │ ← Limit switches (Pins 29,31,33,35)
│ GPIO13 │ GPIO19 │
└─────────────────┘
```

## 🚀 Installation

### 1. Prerequisites
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Dashing (if not already installed)
# Follow official ROS2 Dashing installation guide for Ubuntu 18.04

# Install Jetson.GPIO
sudo pip3 install Jetson.GPIO

# Install additional dependencies
sudo apt install python3-pip python3-yaml
```

### 2. Build the Package
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select stepper_control

# Source the workspace
source install/setup.bash
```

## 🎮 Usage

### 1. Launch the System
```bash
# Launch with default parameters
ros2 launch stepper_control stepper_control.launch.py

# Launch with custom GPIO pins
ros2 launch stepper_control stepper_control.launch.py \
    azimuth_step_pin:=11 \
    azimuth_dir_pin:=13 \
    azimuth_enable_pin:=15 \
    elevator_step_pin:=16 \
    elevator_dir_pin:=18 \
    elevator_enable_pin:=22
```

### 2. Keyboard Controls
Once the system is running, use the following keys:

| Key | Action |
|-----|--------|
| **←** (Left Arrow) | Azimuth motor counter-clockwise |
| **→** (Right Arrow) | Azimuth motor clockwise |
| **↑** (Up Arrow) | Elevator motor up |
| **↓** (Down Arrow) | Elevator motor down |
| **Space** or **Enter** | Emergency stop |
| **q** | Quit the system |

### 3. ROS2 Topics

#### Publishers
- `/stepper/status` (std_msgs/String): System status messages
- `/stepper/joint_states` (sensor_msgs/JointState): Motor position feedback
- `/safety/status` (std_msgs/String): Safety monitor status

#### Subscribers
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/emergency_stop` (std_msgs/String): Emergency stop commands

#### Services
- `/stepper/get_position` (std_srvs/Trigger): Get current motor positions

### 4. Monitoring
```bash
# Monitor system status
ros2 topic echo /stepper/status

# Monitor joint states
ros2 topic echo /stepper/joint_states

# Monitor safety status
ros2 topic echo /safety/status

# Get current position
ros2 service call /stepper/get_position std_srvs/Trigger
```

## ⚙️ Configuration

### Parameter Tuning
Edit `config/motor_params.yaml` or pass parameters via launch arguments:

```bash
# Example: Adjust step delay for smoother motion
ros2 launch stepper_control stepper_control.launch.py step_delay:=0.0005

# Example: Increase steps per command for faster movement
ros2 launch stepper_control stepper_control.launch.py steps_per_command:=20

# Example: Adjust position limits
ros2 launch stepper_control stepper_control.launch.py \
    max_azimuth_position:=1500 \
    min_azimuth_position:=-1500 \
    max_elevator_position:=750 \
    min_elevator_position:=-750
```

### Safety Parameters
- `max_velocity`: Maximum allowed velocity (steps/second)
- `max_azimuth_position`/`min_azimuth_position`: Azimuth position limits
- `max_elevator_position`/`min_elevator_position`: Elevator position limits

## 🔍 Troubleshooting

### Common Issues

#### 1. GPIO Permission Errors
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER

# Set GPIO permissions
sudo chmod 666 /dev/gpiochip0

# Reboot or log out/in
sudo reboot
```

#### 2. Motors Not Moving
- Check GPIO pin connections
- Verify motor driver power supply
- Check enable pin states
- Verify step delay timing

#### 3. Keyboard Not Responding
- Ensure terminal has focus
- Check for conflicting keyboard handlers
- Verify Jetson.GPIO installation

#### 4. Emergency Stop Issues
- Check limit switch connections
- Verify safety parameter settings
- Monitor safety status topic

### Debug Commands
```bash
# Check GPIO pin states
sudo cat /sys/kernel/debug/gpio

# Monitor system resources
htop

# Check ROS2 node status
ros2 node list
ros2 node info /stepper_control_node

# View node logs
ros2 run stepper_control stepper_node --ros-args --log-level DEBUG
```

## 🧪 Testing

### 1. Hardware Test
```bash
# Test individual motor drivers
python3 src/stepper_control/stepper_control/motor_driver.py

# Test keyboard controller
python3 src/stepper_control/stepper_control/keyboard_controller.py
```

### 2. ROS2 Test
```bash
# Test velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

# Test emergency stop
ros2 topic pub /emergency_stop std_msgs/String "data: 'STOP'"
```

## 📋 Safety Guidelines

1. **Always test in a safe environment** before full operation
2. **Keep emergency stop accessible** during testing
3. **Monitor position limits** to prevent mechanical damage
4. **Check motor temperatures** during extended operation
5. **Verify limit switch functionality** before autonomous operation
6. **Use appropriate power supplies** for your motor specifications

## 🔧 Customization

### Adding New Motor Types
1. Extend `MotorType` enum in `motor_driver.py`
2. Add GPIO pin configuration
3. Implement motor-specific control methods
4. Update keyboard mappings if needed

### Modifying Control Algorithms
1. Edit `_cmd_vel_callback` in `stepper_node.py`
2. Adjust velocity-to-step conversion
3. Implement advanced trajectory planning
4. Add PID control loops

## 📚 API Reference

### MotorDriver Class
- `move_azimuth(direction, steps)`: Move azimuth motor
- `move_elevator(direction, steps)`: Move elevator motor
- `get_positions()`: Get current positions
- `set_emergency_stop(stop)`: Control emergency stop
- `cleanup()`: Clean up GPIO resources

### KeyboardController Class
- `start()`: Start keyboard monitoring
- `stop()`: Stop keyboard monitoring
- `set_command_callback(callback)`: Set command handler

### SafetyMonitor Class
- `reset_safety_state()`: Reset safety violations
- `get_safety_status()`: Get safety information

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly on Jetson Nano
5. Submit a pull request

## 📄 License

This project is licensed under the Apache License 2.0.

## 🆘 Support

For issues and questions:
1. Check the troubleshooting section
2. Review ROS2 Dashing documentation
3. Check Jetson.GPIO documentation
4. Open an issue on the repository

---

**Note**: This system is specifically designed for ROS2 Dashing on Ubuntu 18.04 with Jetson Nano hardware. Do not attempt to use with newer ROS2 versions or different hardware without significant modifications. 