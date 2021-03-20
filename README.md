# odrive-ros2-node
ROS2-Node for ODrive

# Features:
- polls odrive velocity, motor_position, applied voltage, odrive temperature and applied torque
- sets desired velocity
- Uses Checksum to communicate with ODrive

# Inner workings
- node polls one value every 10ms
- inner counter spins from 0 to 100
- request rate uses modulo to manage polling order
## example
- rate_bus_voltage = 1, rate_temperature = 2
- output:
  - bus_voltage
  - bus_voltage
  - temperature
  - bus_voltage
  - bus_voltage
  - temperature


# Topics
publishes:
- <std_msgs/Float32>"/odrive_velocity"
- <std_msgs/Float32>"/odrive_position"
- <std_msgs/Float32>"/odrive_bus_voltage"
- <std_msgs/Float32>"/odrive_temperature"
- <std_msgs/Float32>"/odrive_torque"

listens to:
- <std_msgs/Float32>"/odrive_set_velocity"

# Usage
- ros2 run odrive_node odrive_node
- (set to defaults)
- - ros2 run odrive_node odrive_node --ros-args --remap __ns:=/<your-namespace> -p port:='/dev/ttyS1' -p motor:=0 -p rate_position_velocity:=1  -p rate_bus_voltage:=2 -p rate_temperature:=3 -p rate_torque:=4

# Dependencies
- rclcpp
- std_msgs
