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
#include "rclcpp_components/register_node_macro.hpp" /*** Step 1: This is the first step 
in changing this node  a component node, it should be placed after the "rclcpp/rclcpp.hpp" include ***/

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/*** Step 2: The class should be put in a namespace ***/
// The name of the namespace and the package should be thesame 
namespace my_composition_package{

class MinimalPublisher : public rclcpp::Node // this class is a subclass of the rclcpp::Node class
{
public:
  /*** Step 3: Use a constructor with the NodeOptions argument ***/
  MinimalPublisher(const rclcpp::NodeOptions & options)
  : Node("minimal_publisher", options), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

}
/*** Step 5: Register the node as a component with the code below ***/
RCLCPP_COMPONENTS_REGISTER_NODE(my_composition_package::MinimalPublisher) 

/*** Step 4: Remove the main function ***/

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }
