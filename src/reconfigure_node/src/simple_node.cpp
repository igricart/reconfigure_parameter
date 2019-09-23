
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class SimpleNode : public rclcpp::Node
{
public:
  SimpleNode()
  : Node("simple_node",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
        .start_parameter_event_publisher(true)
    ),
    count_(0)
  {
    std::cout << "Constructor called" << std::endl;

    auto timer_callback =
      [this]() -> void {this->sendParameters();};
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

  void sendParameters()
  {
    auto client = this->create_client<rcl_interfaces::srv::SetParameters>(std::string(this->get_name()) + "/set_parameters");

    auto param_names = this->list_parameters(
      {},
      rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE
    );

    std::vector<rclcpp::Parameter> params;
    std::vector<std::string> params_names_vector(param_names.names.size());

    std::cout << "Parameters in server: " << param_names.names.size() << std::endl;
    for(size_t i = 0; i < param_names.names.size(); ++i)
      params_names_vector[i] = param_names.names[i];

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto param = rcl_interfaces::msg::Parameter();

    std::cout << "After client declaration" << std::endl;
    for(auto& v : params_names_vector)
    {
      std::cout << this->get_parameter(v).get_name() << " - " << this->get_parameter(v).value_to_string() << std::endl;
      param.name = this->get_parameter(v).get_name();
      param.value = this->get_parameter(v).get_value_message();
      request->parameters.push_back(param);
    }

    client->async_send_request(request);
    this->timer_->cancel();
    rclcpp::shutdown();
  }

private:
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleNode>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node);
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
