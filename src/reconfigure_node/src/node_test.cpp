
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    std::cout << "Constructor called" << std::endl;
    this->declare_parameter<int>("asdf", 10);
    this->declare_parameter<int>("fdsa", 20);
    this->declare_parameter<int>("my_value", 10);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "my_value: '%i'", this->get_parameter("my_value").as_int());
        RCLCPP_INFO(this->get_logger(), "asdf: '%i'", this->get_parameter("asdf").as_int());
        RCLCPP_INFO(this->get_logger(), "fdsa: '%i'", this->get_parameter("fdsa").as_int());

        if(count_ == 1)
          this->send_parameters();

      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }
  void send_parameters()
  {
    // if (this->get_parameter("my_value").as_int() != 0)
    // {

      RCLCPP_INFO(this->get_logger(), "AAA");
      
      auto my_value = this->get_parameter("my_value").as_int();
      this->set_parameter({"asdf", my_value });

      RCLCPP_INFO(this->get_logger(), "BBB");
      
      auto client = this->create_client<rcl_interfaces::srv::SetParameters>("/minimal_publisher/set_parameters");
      auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      auto param = rcl_interfaces::msg::Parameter();
      param.name = "fdsa";
      param.value.type = 2;
      param.value.integer_value = my_value;
      request->parameters.push_back(param);

      RCLCPP_INFO(this->get_logger(), "CCC");
      
      client->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "DDD");
    // }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node);
  exe.spin_some();
  // node->send_parameters();
  exe.spin();
  // exe.spin_node_some(node->get_node_base_interface());
  // rclcpp::spin(mynode);
  rclcpp::shutdown();
  return 0;
}
