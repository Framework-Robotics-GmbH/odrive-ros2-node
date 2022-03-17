#include "odrive_node.hpp"

ODriveNode::ODriveNode() : Node("odrive_node") {
    std::string port = "/dev/ttyS1";
    counter = 0;
    set_r_speed = 0.0;
    set_l_speed = 0.0;

    // request parameter from Node
    this->declare_parameter<std::string>("port", port);
    this->declare_parameter<int>("motor", 0);
    this->declare_parameter<int>("priority_position_velocity", 1);
    this->declare_parameter<int>("priority_bus_voltage", 2);
    this->declare_parameter<int>("priority_temperature", 3);
    this->declare_parameter<int>("priority_torque", 4);
    this->declare_parameter<double>("wheel_dist", 1);
    this->declare_parameter<double>("gear_ratio", 1);

    // node sets parameter, if given
    this->get_parameter("port", port);
    this->get_parameter("motor", motor);
    this->get_parameter("priority_position_velocity", priority_position_velocity);
    this->get_parameter("priority_bus_voltage", priority_bus_voltage);
    this->get_parameter("priority_temperature", priority_temperature);
    this->get_parameter("priority_torque", priority_torque);
    this->get_parameter("wheel_dist", wheel_dist);
    this->get_parameter("gear_ratio", gear_ratio);

    odrive = new ODrive(port, this);

    // create topic, if priority != 0, setup motor = 0, motor = 1, or both if motor = 2
    if (priority_position_velocity) {
        if (motor == 2 || motor == 0) {
            publisher_velocity0 = this->create_publisher<std_msgs::msg::Float32>(
                    "odrive" + std::to_string(0) + "_velocity", 10);
            publisher_position0 = this->create_publisher<std_msgs::msg::Float32>(
                    "odrive" + std::to_string(0) + "_position", 10);
        }
        if (motor == 2 || motor == 1) {
            publisher_velocity1 = this->create_publisher<std_msgs::msg::Float32>(
                    "odrive" + std::to_string(1) + "_velocity", 10);
            publisher_position1 = this->create_publisher<std_msgs::msg::Float32>(
                    "odrive" + std::to_string(1) + "_position", 10);
        }
    }

    if (priority_torque) {
        if (motor == 2 || motor == 0) {
            publisher_torque0 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(0) + "_torque",
                                                                               10);
        }
        if (motor == 2 || motor == 1) {
            publisher_torque1 = this->create_publisher<std_msgs::msg::Float32>("odrive" + std::to_string(1) + "_torque",
                                                                               10);
        }
    }

    if (priority_bus_voltage) {
        publisher_bus_voltage = this->create_publisher<std_msgs::msg::Float32>("odrive_bus_voltage", 10);
    }

    if (priority_temperature) {
        if (motor == 2 || motor == 0) {
            publisher_temperature0 = this->create_publisher<std_msgs::msg::Float32>(
                    "odrive" + std::to_string(0) + "_temperature", 10);
        }
        if (motor == 2 || motor == 1) {
            publisher_temperature1 = this->create_publisher<std_msgs::msg::Float32>(
                    "odrive" + std::to_string(1) + "_temperature", 10);
        }
    }

    if (motor == 2 || motor == 0) {
        subscription_velocity0 = this->create_subscription<std_msgs::msg::Float32>(
                "odrive" + std::to_string(0) + "_set_velocity", 10,
                std::bind(&ODriveNode::velocity_callback0, this, std::placeholders::_1));
    }
    if (motor == 2 || motor == 1) {
        subscription_velocity1 = this->create_subscription<std_msgs::msg::Float32>(
                "odrive" + std::to_string(1) + "_set_velocity", 10,
                std::bind(&ODriveNode::velocity_callback1, this, std::placeholders::_1));
    }
    if (motor == 2) {
        subscription_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("odrive_cmd_velocity", 10,
                                                                                    std::bind(
                                                                                            &ODriveNode::cmd_velocity_callback,
                                                                                            this,
                                                                                            std::placeholders::_1));
//        publisher_odom_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("odrive_odom_velocity", 10);
        publisher_odom_vel = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("odrive_odom_velocity", 10);
    }


//set callback rate
    timer_ = this->create_wall_timer(10ms, std::bind(&ODriveNode::odrive_callback, this));
}

ODriveNode::~ODriveNode() {
    odrive->setVelocity(0, 0);
    odrive->setVelocity(1, 0);
    delete (odrive);
}

// send velocity to odrive motor 0
void ODriveNode::velocity_callback0(const std_msgs::msg::Float32::SharedPtr msg) {
    float velocity = msg->data;
    odrive->setVelocity(0, velocity / gear_ratio);
}

// send velocity to odrive motor 1
void ODriveNode::velocity_callback1(const std_msgs::msg::Float32::SharedPtr msg) {
    float velocity = msg->data;
    odrive->setVelocity(1, velocity / gear_ratio);
}

// send velocity on both motors from Twist Message
void ODriveNode::cmd_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    Vector3 angular = msg->angular;
    Vector3 linear = msg->linear;
    double vel = linear.x;
    double angle = angular.z;

    double r_speed = ((angle * wheel_dist) / 2 + vel);
    double l_speed = (vel * 2.0 - r_speed);
    std::cout << "Asked r_speed: " << r_speed << " l_speed: " << l_speed;
    r_speed /= gear_ratio;
    l_speed /= gear_ratio;
    std::cout << " Aplied (" << gear_ratio << "x gear ratio) r_speed: " << r_speed << " l_speed: " << l_speed
              << std::endl;
    set_r_speed = r_speed;
    set_l_speed = l_speed;
    odrive->setVelocity(0, r_speed);
    odrive->setVelocity(1, l_speed);
}

void ODriveNode::odrive_callback() {
    // start one request at a time, skip if priority = 0
    if (!(order[0] || order[1] || order[2] || order[3])) {
        if (priority_position_velocity && (counter % priority_position_velocity == 0))
            order[0] = 1;
        if (priority_bus_voltage && (counter % priority_bus_voltage == 0))
            order[1] = 1;
        if (priority_temperature && (counter % priority_temperature == 0))
            order[2] = 1;
        if (priority_torque && (counter % priority_torque == 0))
            order[3] = 1;
        ++counter %= 100;
    }

    // publish position & velocity
    if (order[0]) {
        order[0] = 0;
        std::pair<float, float> values;
        auto position_msg = std_msgs::msg::Float32();
        auto velocity_msg = std_msgs::msg::Float32();
        if (motor == 2 || motor == 0) {
            values = odrive->getPosition_Velocity(0);
            if (values.first != -1.0) {
                position_msg.data = values.first * gear_ratio;
                publisher_position0->publish(position_msg);
                velocity_msg.data = values.second * gear_ratio;
                publisher_velocity0->publish(velocity_msg);
            }
        }
        if (motor == 2 || motor == 1) {
            values = odrive->getPosition_Velocity(1);
            if (values.first != -1.0) {
                position_msg.data = values.first * gear_ratio;
                publisher_position1->publish(position_msg);
                velocity_msg.data = values.second * gear_ratio;
                publisher_velocity1->publish(velocity_msg);
            }
        }
        if (motor == 2) {
//            auto odom_msg = geometry_msgs::msg::TwistStamped();
            auto odom_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
//        double r_speed = odrive->getPosition_Velocity(0).second;
//        double l_speed = odrive->getPosition_Velocity(1).second;
            double r_speed = set_r_speed;
            double l_speed = set_l_speed;
            r_speed *= gear_ratio;
            l_speed *= gear_ratio;

            odom_msg.header.stamp = this->get_clock()->now();
            // from the nav2 tutorial
            odom_msg.twist.twist.linear.x = (r_speed + l_speed) / 2;
            odom_msg.twist.twist.angular.z = (r_speed - l_speed) / wheel_dist;
            // computed by hand
//        odom_msg.twist.linear.x = (r_speed + l_speed) / 2;
//        odom_msg.twist.angular.z = (-2 * l_speed) / wheel_dist;
            publisher_odom_vel->publish(odom_msg);
        }
    }
        // publish bus voltage
    else if (order[1]) {
        order[1] = 0;
        float voltage = odrive->getBusVoltage();
        if (voltage != -1.0) {
            auto voltage_msg = std_msgs::msg::Float32();
            voltage_msg.data = voltage;
            publisher_bus_voltage->publish(voltage_msg);
        }
    }
        // publish temperature
    else if (order[2]) {
        order[2] = 0;
        float temperature;
        auto temperature_msg = std_msgs::msg::Float32();
        if (motor == 2 || motor == 0) {
            temperature = odrive->getTemperature(0);
            if (temperature != -1.0) {
                temperature_msg.data = temperature;
                publisher_temperature0->publish(temperature_msg);
            }
        }
        if (motor == 2 || motor == 1) {
            temperature = odrive->getTemperature(1);
            if (temperature != -1.0) {
                temperature_msg.data = temperature;
                publisher_temperature1->publish(temperature_msg);
            }
        }
    }
        // publish torque
    else if (order[3]) {
        order[3] = 0;
        float torque;
        auto torque_msg = std_msgs::msg::Float32();
        if (motor == 2 || motor == 0) {
            torque = odrive->getTorque(0);
            if (torque != -1.0) {
                torque_msg.data = torque;
                publisher_torque0->publish(torque_msg);
            }
        }
        if (motor == 2 || motor == 1) {
            torque = odrive->getTorque(1);
            if (torque != -1.0) {
                torque_msg.data = torque;
                publisher_torque1->publish(torque_msg);
            }
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODriveNode>());
    rclcpp::shutdown();
    return 0;
}
