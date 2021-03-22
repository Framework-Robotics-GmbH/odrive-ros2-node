# odrive-ros2-node
ROS2-Node for ODrive

# Features:
- polls odrive velocity, motor_position, applied voltage, odrive temperature and applied torque
- sets desired velocity
- Uses Checksum when communicating with ODrive

# Inner workings
- node polls one value every 10ms
- request rate sets priority in poll-list using modulo(0 - 99)

## example
- rate_bus_voltage = 1, rate_temperature = 2
- output:
- counter 0:
  - (@10ms)bus_voltage
- counter 1:
  - (@20ms)bus_voltage
  - (@30ms)temperature
- counter 3:
  - (@40ms)bus_voltage
- counter 4:
  - (@50ms)bus_voltage
  - (@60ms)temperature
- ...

# Topics
- publishes:


- - <std_msgs/Float32>"/odrive_bus_voltage"
- - <std_msgs/Float32>"/odrive0_velocity"
- - <std_msgs/Float32>"/odrive0_position"
- - <std_msgs/Float32>"/odrive0_temperature"
- - <std_msgs/Float32>"/odrive0_torque"


- - <std_msgs/Float32>"/odrive1_velocity"
- - <std_msgs/Float32>"/odrive1_position"
- - <std_msgs/Float32>"/odrive1_temperature"
- - <std_msgs/Float32>"/odrive1_torque"
- set rate to 0 to disable topic

listens to:
- <std_msgs/Float32>"/odrive0_set_velocity"
- <std_msgs/Float32>"/odrive1_set_velocity"

# Usage
- ros2 run odrive_node odrive_node
- set motor variable to 0(default), 1, or 2 to set both to active
- set rate from 0 to 99
- (set to defaults)
- - ros2 run odrive_node odrive_node --ros-args --remap __ns:=/<your-namespace> -p port:='/dev/ttyS1' -p motor:=0 -p rate_position_velocity:=1  -p rate_bus_voltage:=2 -p rate_temperature:=3 -p rate_torque:=4

# Dependencies
- rclcpp
- std_msgs
