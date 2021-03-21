#include "odrive_node.hpp"

ODriveNode::ODriveNode() : Node("odrive_node")
{
  std::string port = "/dev/ttyS1";
  counter = 0;

  // request parameter from Node
  this->declare_parameter<std::string>("port", port);
  this->declare_parameter<int>("motor", 0);
  this->declare_parameter<int>("rate_position_velocity", 1);
  this->declare_parameter<int>("rate_bus_voltage", 2);
  this->declare_parameter<int>("rate_temperature", 3);
  this->declare_parameter<int>("rate_torque", 4);

  // node sets parameter, if given
  this->get_parameter("port", port);
  this->get_parameter("motor", motor);
  this->get_parameter("rate_position_velocity", rate_position_velocity);
  this->get_parameter("rate_bus_voltage", rate_bus_voltage);
  this->get_parameter("rate_temperature", rate_temperature);
  this->get_parameter("rate_torque", rate_torque);

  odrive = new ODrive(port, this);

  // create topic, if rate != 0, setup motor = 0, motor = 1, or both if motor = 2
  if (rate_position_velocity)
  {
    if (motor == 2 || motor == 0)
    {
      publisher_velocity0 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(0) + "_velocity", 10);
      publisher_position0 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(0) + "_position", 10);
    }
    if (motor == 2 || motor == 1)
    {
      publisher_velocity1 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(1) + "_velocity", 10);
      publisher_position1 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(1) + "_position", 10);
    }
  }

  if (rate_torque)
  {
    if (motor == 2 || motor == 0)
    {
      publisher_torque0 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(0) + "_torque", 10);
    }
    if (motor == 2 || motor == 1)
    {
      publisher_torque1 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(1) + "_torque", 10);
    }
  }

  if (rate_bus_voltage)
  {
    publisher_bus_voltage = this->create_publisher<std_msgs::msg::Float32>("odrive_bus_voltage", 10);
  }

  if (rate_temperature)
  {
    if (motor == 2 || motor == 0)
    {
      publisher_temperature0 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(0) + "_temperature", 10);
    }
    if (motor == 2 || motor == 1)
    {
      publisher_temperature1 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(1) + "_temperature", 10);
    }
  }

  if (motor == 2 || motor == 0)
  {
    subscription_velocity0 = this->create_subscription<std_msgs::msg::Float32>("odrive" + std::to_string(0) + "_set_velocity", 10, std::bind(&ODriveNode::velocity_callback0, this, std::placeholders::_1));
  }
  if (motor == 2 || motor == 1)
  {
    subscription_velocity0 = this->create_subscription<std_msgs::msg::Float32>("odrive" + std::to_string(1) + "_set_velocity", 10, std::bind(&ODriveNode::velocity_callback1, this, std::placeholders::_1));
  }

//set callback rate
  timer_ = this->create_wall_timer(10ms, std::bind(&ODriveNode::odrive_callback, this));
}

ODriveNode::~ODriveNode()
{
  delete (odrive);
}

// send velocity to odrive motor 0
void ODriveNode::velocity_callback0(const std_msgs::msg::Float32::SharedPtr msg)
{
  float velocity = msg->data;
  odrive->setVelocity(0, velocity);
}

// send velocity to odrive motor 1
void ODriveNode::velocity_callback1(const std_msgs::msg::Float32::SharedPtr msg)
{
  float velocity = msg->data;
  odrive->setVelocity(1, velocity);
}

void ODriveNode::odrive_callback()
{
  // start one request at a time, skip if rate = 0
  if (!(order[0] || order[1] || order[2] || order[3]))
  {
    if (rate_position_velocity && (counter % rate_position_velocity == 0))
      order[0] = 1;
    if (rate_bus_voltage && (counter % rate_bus_voltage == 0))
      order[1] = 1;
    if (rate_temperature && (counter % rate_temperature == 0))
      order[2] = 1;
    if (rate_torque && (counter % rate_torque == 0))
      order[3] = 1;
    ++counter %= 100;
  }

  // publish position & velocity
  if (order[0])
  {
    order[0] = 0;
    std::pair<float, float> values;
    auto position_msg = std_msgs::msg::Float32();
    auto velovity_msg = std_msgs::msg::Float32();
    if (motor == 2 || motor == 0)
    {
      values = odrive->getPosition_Velocity(0);
      if (values.first != -1.0)
      {
        position_msg.data = values.first;
        publisher_position0->publish(position_msg);
        velovity_msg.data = values.second;
        publisher_velocity0->publish(velovity_msg);
      }
    }
    if (motor == 2 || motor == 1)
    {
      values = odrive->getPosition_Velocity(1);
      if (values.first != -1.0)
      {
        position_msg.data = values.first;
        publisher_position1->publish(position_msg);
        velovity_msg.data = values.second;
        publisher_velocity1->publish(velovity_msg);
      }
    }
  }
  // publish bus voltage
  else if (order[1])
  {
    order[1] = 0;
    float voltage = odrive->getBusVoltage();
    if (voltage != -1.0)
    {
      auto voltage_msg = std_msgs::msg::Float32();
      voltage_msg.data = voltage;
      publisher_bus_voltage->publish(voltage_msg);
    }
  }
  // publish temperature
  else if (order[2])
  {
    order[2] = 0;
    float temperature;
    auto temperature_msg = std_msgs::msg::Float32();
    if (motor == 2 || motor == 0)
    {
      temperature = odrive->getTemperature(0);
      if (temperature != -1.0)
      {
        temperature_msg.data = temperature;
        publisher_temperature0->publish(temperature_msg);
      }
    }
    if (motor == 2 || motor == 1)
    {
      temperature = odrive->getTemperature(1);
      if (temperature != -1.0)
      {
        temperature_msg.data = temperature;
        publisher_temperature1->publish(temperature_msg);
      }
    }
  }
  // publish torque
  else if (order[3])
  {
    order[3] = 0;
    float torque;
    auto torque_msg = std_msgs::msg::Float32();
    if (motor == 2 || motor == 0)
    {
      torque = odrive->getTorque(0);
      if (torque != -1.0)
      {
        torque_msg.data = torque;
        publisher_torque0->publish(torque_msg);
      }
    }
    if (motor == 2 || motor == 1)
    {
      torque = odrive->getTorque(1);
      if (torque != -1.0)
      {
        torque_msg.data = torque;
        publisher_torque1->publish(torque_msg);
      }
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
