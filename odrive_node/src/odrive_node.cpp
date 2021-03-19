#include "odrive_node.hpp"

ODriveNode::ODriveNode(const std::string port, const int motor, const int rate_position_velocity, const int rate_bus_voltage, const int rate_temperature, const int rate_torque)
    : Node("odrive_node"), odrive(ODrive(port, this)), motor(motor), rate_position_velocity(rate_position_velocity), rate_bus_voltage(rate_bus_voltage), rate_temperature(rate_temperature), rate_torque(rate_torque)
{
  counter = 0;
  publisher_velocity = this->create_publisher<std_msgs::msg::Float32>("odrive_velocity", 10);
  publisher_position = this->create_publisher<std_msgs::msg::Float32>("odrive_position", 10);
  publisher_bus_voltage = this->create_publisher<std_msgs::msg::Float32>("odrive_bus_voltage", 10);
  publisher_temperature = this->create_publisher<std_msgs::msg::Float32>("odrive_temperature", 10);
  publisher_torque = this->create_publisher<std_msgs::msg::Float32>("odrive_torque", 10);
  subscription_velocity = this->create_subscription<std_msgs::msg::Float32>("odrive_set_velocity", 10, std::bind(&ODriveNode::velocity_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(10ms, std::bind(&ODriveNode::odrive_callback, this));
}

// send velocity to odrive
void ODriveNode::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  float velocity = msg->data;
  odrive.setVelocity(motor, velocity);
}

void ODriveNode::odrive_callback()
{
  // start one request at a time
  if (!(order[0] || order[1] || order[2] || order[3]))
  {
    if (counter % rate_position_velocity == 0)
      order[0] = 1;
    if (counter % rate_bus_voltage == 0)
      order[1] = 1;
    if (counter % rate_temperature == 0)
      order[2] = 1;
    if (counter % rate_torque == 0)
      order[3] = 1;
    ++counter %= 100;
  }

  if (order[0])
  {
    order[0] = 0;
    auto values = odrive.getPosition_Velocity(0);
    if (values.first != -1.0)
    {
      auto position_msg = std_msgs::msg::Float32();
      position_msg.data = values.first;
      publisher_position->publish(position_msg);
      auto velovity_msg = std_msgs::msg::Float32();
      velovity_msg.data = values.second;
      publisher_velocity->publish(velovity_msg);
    }
  }
  else if (order[1])
  {
    order[1] = 0;
    float voltage = odrive.getBusVoltage();
    if (voltage != -1.0)
    {
      auto voltage_msg = std_msgs::msg::Float32();
      voltage_msg.data = voltage;
      publisher_bus_voltage->publish(voltage_msg);
    }
  }
  else if (order[2])
  {
    order[2] = 0;
    float temperature = odrive.getTemperature(0);
    if (temperature != -1.0)
    {
      auto temperature_msg = std_msgs::msg::Float32();
      temperature_msg.data = temperature;
      publisher_temperature->publish(temperature_msg);
    }
  }
  else if (order[3])
  {
    order[3] = 0;
    float torque = odrive.getTorque(0);
    if (torque != -1.0)
    {
      auto torque_msg = std_msgs::msg::Float32();
      torque_msg.data = torque;
      publisher_torque->publish(torque_msg);
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODriveNode>());
  rclcpp::shutdown();
  return 0;
}
