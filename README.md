# odrive-ros2-node

ROS2-Nodes for ODrive

# Features

- Polls odrive velocity, motor_position, applied voltage, odrive temperature and applied torque
- Sets desired velocity
    - Directly or from `geometry_msgs.msg.Twist`
- Uses Checksum when communicating with ODrive
- Use wheel dist, gear ratio and wheel circumference
- Odrive init/calibration node
- Shares transformations and messages required by the **Nav2 stack**   
- Launch file

# Start with launch file
```bash
ros2 launch odrive_odom odrive_launch.py
```

# `odrive_node`
## Polling rate

- node polls one value every 10ms
- request priority sets priority in poll-list using modulo (0 -- 99)

### Example

- priority_bus_voltage = 1, priority_temperature = 2
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

## Topics

- publishes:

* - <std_msgs/Float32>"/odrive_bus_voltage"
* - <std_msgs/Float32>"/odrive0_velocity"
* - <std_msgs/Float32>"/odrive0_position"
* - <std_msgs/Float32>"/odrive0_temperature"
* - <std_msgs/Float32>"/odrive0_torque"

- - <std_msgs/Float32>"/odrive1_velocity"
- - <std_msgs/Float32>"/odrive1_position"
- - <std_msgs/Float32>"/odrive1_temperature"
- - <std_msgs/Float32>"/odrive1_torque"

* - <geometry_msgs/TwistStamped>"/odrive_odom_velocity"
- set priority to 0 to disable topic

listens to:

- <std_msgs/Float32>"/odrive0_set_velocity"
- <std_msgs/Float32>"/odrive1_set_velocity"
- <geometry_msgs/Twist>"/odrive_cmd_velocity"

## Usage

- ros2 run odrive_node odrive_node
- set motor variable to 0 (default), 1, or 2 to set both to active
- set priority from 0 to 99 
- set `wheel_dist` [meters]
- set `gear_ratio` [meters/rad] = `gear ratio * wheel circumference`
- (set to defaults)
- - ros2 run odrive_node odrive_node --ros-args --remap __ns:=/<your-namespace> -p port:='/dev/ttyS1' -p motor:=0 -p priority_position_velocity:=1 -p priority_bus_voltage:=2 -p priority_temperature:=3 -p priority_torque:=4

## Dependencies

- rclcpp
- std_msgs
- geometry_msgs

# `odrive_odom`

- Initialize Odrive
- - `ros2 run odrive_odom init`
- Publish encoder odometry
- - `ros2 run odrive_odom odom`
