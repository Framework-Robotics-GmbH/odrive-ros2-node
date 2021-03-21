/* BSD 3-Clause License

Copyright (c) 2021, Framework Robotics GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ODrive.cpp"

using namespace std::chrono_literals;
class ODriveNode : public rclcpp::Node
{
public:
    ODriveNode();
    ~ODriveNode();

private:
    ODrive *odrive;
    int rate_position_velocity ;
    int rate_bus_voltage;
    int rate_temperature;
    int rate_torque;
    int counter;
    int motor;
    int order[4] = {0, 0, 0, 0};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_velocity0;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_velocity1;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_position0;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_position1;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_torque0;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_torque1;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_bus_voltage;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_temperature0;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_temperature1;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_velocity0;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_velocity1;
    void odrive_callback();
    void velocity_callback0(const std_msgs::msg::Float32::SharedPtr msg);
    void velocity_callback1(const std_msgs::msg::Float32::SharedPtr msg);
};
