# LBR Demos
Demos for controlling the LBR through the Fast Robot Interface (FRI) from ROS 2.

⚠️ **General Safety**: Do always execute in `T1` mode on the real robot first.

## LBR FRI ROS 2 Demos
🤝 **Note**: These demos are compatible with and closely follow KUKA's FRI example applications.

🫣 **Note**: A real robot is required to run these demos.

### Python Demos
[Python examples](lbr_fri_ros2_python_demos/) for sending commands to `/lbr_command` and reading states from `/lbr_state`.

### Advanced Python Demos
[Advanced Python examples](lbr_fri_ros2_advanced_python_demos/) for sending commands to `/lbr_command` and reading states from `/lbr_state`.

### C++ Demos
[C++ examples](lbr_fri_ros2_cpp_demos/) examples for sending commands to `/lbr_command` and reading states from `/lbr_state`.

## LBR ROS2 Control Demos
These demos demonstrate the LBR integration into the ROS 2 ecosystem. 

🙌 **Note**: These demos run in simulation **and** on the real robot.

### Python Demos
[Python examples](lbr_fri_ros2_python_demos/) for using the LBR through `ros2_control` controllers.

### C++ Demos
[C++ examples](lbr_fri_ros2_cpp_demos/) for using the LBR through `ros2_control` controllers.
