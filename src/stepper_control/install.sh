#!/bin/bash
# Installation Script for Jetson Nano Stepper Motor Control System
# Compatible with Ubuntu 18.04 and ROS2 Dashing

set -e  # Exit on any error

echo "=========================================="
echo "Jetson Nano Stepper Motor Control System"
echo "Installation Script for Ubuntu 18.04"
echo "=========================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Jetson Nano
check_hardware() {
    print_status "Checking hardware compatibility..."
    
    if [[ -f "/etc/nv_tegra_release" ]]; then
        print_success "Jetson Nano detected"
    else
        print_warning "Not running on Jetson Nano - some features may not work"
    fi
    
    # Check architecture
    ARCH=$(uname -m)
    if [[ "$ARCH" == "aarch64" ]]; then
        print_success "ARM64 architecture detected"
    else
        print_error "Expected ARM64 architecture, found: $ARCH"
        exit 1
    fi
}

# Check Ubuntu version
check_ubuntu_version() {
    print_status "Checking Ubuntu version..."
    
    UBUNTU_VERSION=$(lsb_release -rs)
    if [[ "$UBUNTU_VERSION" == "18.04" ]]; then
        print_success "Ubuntu 18.04 detected"
    else
        print_warning "Expected Ubuntu 18.04, found: $UBUNTU_VERSION"
        print_warning "This system is designed for Ubuntu 18.04"
    fi
}

# Check ROS2 Dashing installation
check_ros2_dashing() {
    print_status "Checking ROS2 Dashing installation..."
    
    if command -v ros2 &> /dev/null; then
        ROS2_VERSION=$(ros2 --version | head -n1)
        if [[ "$ROS2_VERSION" == *"Dashing"* ]]; then
            print_success "ROS2 Dashing detected: $ROS2_VERSION"
        else
            print_warning "ROS2 detected but not Dashing: $ROS2_VERSION"
            print_warning "This system requires ROS2 Dashing"
        fi
    else
        print_error "ROS2 not found. Please install ROS2 Dashing first."
        print_status "Installation guide: https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html"
        exit 1
    fi
}

# Install system dependencies
install_dependencies() {
    print_status "Installing system dependencies..."
    
    sudo apt update
    
    # Install required packages
    sudo apt install -y \
        python3-pip \
        python3-yaml \
        python3-dev \
        build-essential \
        git \
        cmake \
        pkg-config
    
    print_success "System dependencies installed"
}

# Install Jetson.GPIO
install_jetson_gpio() {
    print_status "Installing Jetson.GPIO..."
    
    # Check if Jetson.GPIO is already installed
    if python3 -c "import Jetson.GPIO" 2>/dev/null; then
        print_success "Jetson.GPIO already installed"
    else
        # Install Jetson.GPIO
        sudo pip3 install Jetson.GPIO
        
        if python3 -c "import Jetson.GPIO" 2>/dev/null; then
            print_success "Jetson.GPIO installed successfully"
        else
            print_error "Failed to install Jetson.GPIO"
            exit 1
        fi
    fi
}

# Setup GPIO permissions
setup_gpio_permissions() {
    print_status "Setting up GPIO permissions..."
    
    # Add user to gpio group
    sudo usermod -a -G gpio $USER
    
    # Create udev rules for GPIO access
    sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null <<EOF
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c 'chown -R root:gpio /sys/class/gpio && chmod -R 770 /sys/class/gpio; chown -R root:gpio /sys/devices/virtual/gpio && chmod -R 770 /sys/devices/virtual/gpio; chown -R root:gpio /sys$devpath && chmod -R 770 /sys$devpath'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys/class/gpio/export && chmod 220 /sys/class/gpio/export; chown root:gpio /sys/class/gpio/unexport && chmod 220 /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys%p/active_low && chmod 660 /sys%p/active_low; chown root:gpio /sys%p/direction && chmod 660 /sys%p/direction; chown root:gpio /sys%p/edge && chmod 660 /sys%p/edge; chown root:gpio /sys%p/value && chmod 660 /sys%p/value'"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    print_success "GPIO permissions configured"
}

# Build the ROS2 package
build_package() {
    print_status "Building stepper_control package..."
    
    # Check if we're in a ROS2 workspace
    if [[ ! -f "package.xml" ]] || [[ ! -d "src" ]]; then
        print_error "Not in a ROS2 workspace. Please run this script from your ROS2 workspace root."
        exit 1
    fi
    
    # Build the package
    colcon build --packages-select stepper_control
    
    if [[ $? -eq 0 ]]; then
        print_success "Package built successfully"
    else
        print_error "Failed to build package"
        exit 1
    fi
}

# Create test script
create_test_script() {
    print_status "Creating test script..."
    
    cat > test_stepper_system.py << 'EOF'
#!/usr/bin/env python3
"""
Quick test script for stepper motor system
"""

import sys
import os

# Add the package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def test_imports():
    """Test if all modules can be imported"""
    try:
        from stepper_control.motor_driver import MotorDriver, MotorDirection
        print("✓ Motor driver module imported")
        
        from stepper_control.keyboard_controller import KeyboardController, KeyCommand
        print("✓ Keyboard controller module imported")
        
        from stepper_control.safety_monitor import SafetyMonitor
        print("✓ Safety monitor module imported")
        
        from stepper_control.stepper_node import StepperNode
        print("✓ Stepper node module imported")
        
        return True
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False

def test_gpio():
    """Test GPIO access"""
    try:
        import Jetson.GPIO as GPIO
        print("✓ Jetson.GPIO imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Jetson.GPIO import error: {e}")
        return False

def main():
    print("Testing stepper motor control system...")
    
    success = True
    
    # Test imports
    if not test_imports():
        success = False
    
    # Test GPIO
    if not test_gpio():
        success = False
    
    if success:
        print("\n✓ All tests passed!")
        print("System is ready for use.")
    else:
        print("\n✗ Some tests failed.")
        print("Please check the installation.")

if __name__ == "__main__":
    main()
EOF
    
    chmod +x test_stepper_system.py
    print_success "Test script created"
}

# Create launch script
create_launch_script() {
    print_status "Creating launch script..."
    
    cat > launch_stepper.sh << 'EOF'
#!/bin/bash
# Launch script for stepper motor control system

echo "Launching stepper motor control system..."

# Source ROS2 environment
source /opt/ros/dashing/setup.bash

# Source workspace
source install/setup.bash

# Launch the system
ros2 launch stepper_control stepper_control.launch.py "$@"
EOF
    
    chmod +x launch_stepper.sh
    print_success "Launch script created"
}

# Main installation function
main() {
    echo "Starting installation..."
    
    # Check prerequisites
    check_hardware
    check_ubuntu_version
    check_ros2_dashing
    
    # Install dependencies
    install_dependencies
    install_jetson_gpio
    
    # Setup permissions
    setup_gpio_permissions
    
    # Build package
    build_package
    
    # Create utility scripts
    create_test_script
    create_launch_script
    
    print_success "Installation completed successfully!"
    
    echo ""
    echo "=========================================="
    echo "Installation Summary:"
    echo "=========================================="
    echo "✓ Hardware compatibility verified"
    echo "✓ System dependencies installed"
    echo "✓ Jetson.GPIO installed"
    echo "✓ GPIO permissions configured"
    echo "✓ ROS2 package built"
    echo "✓ Test script created"
    echo "✓ Launch script created"
    echo ""
    echo "Next steps:"
    echo "1. Reboot or log out/in for GPIO permissions to take effect"
    echo "2. Run: ./test_stepper_system.py"
    echo "3. Run: ./launch_stepper.sh"
    echo "4. Use arrow keys to control motors"
    echo ""
    echo "For more information, see README.md"
    echo "=========================================="
}

# Run main function
main "$@" 