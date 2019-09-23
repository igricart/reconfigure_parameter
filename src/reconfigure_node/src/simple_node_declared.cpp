
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
  : Node("simple_node_declared",
      rclcpp::NodeOptions()
        .start_parameter_event_publisher(true)
    ),
    count_(0)
  {
    std::cout << "Constructor called" << std::endl;
    this->declare_parameter<int>("my_value", 100);
    this->declare_parameter<std::string>("a_string", "initial_value");
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

    std::cout << "After client declaration" << std::endl; 
    for(auto& v : params_names_vector)
      std::cout << this->get_parameter(v).get_name() << " - " << this->get_parameter(v).value_to_string() << std::endl;

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    auto param = rcl_interfaces::msg::Parameter();
    param.name = "my_value";
    param.value.type = 2;
    param.value.integer_value = this->get_parameter("my_value").as_int();
    request->parameters.push_back(param);    
    client->async_send_request(request);
    std::cout << "After sending request" << std::endl;
    std::cout << this->get_name() << " - " << this->get_parameter("my_value").as_int() << std::endl;
  }

private:
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleNode>();
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
