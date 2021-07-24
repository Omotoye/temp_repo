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
#include <cinttypes>
#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

class MinimalClient : public rclcpp::Node
{
public:
  
  MinimalClient()
  : Node("minimal_client")
  {
    client_ = this->create_client<AddTwoInts>("add_two_ints");
    while (!client_->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }
  this->request_ = std::make_shared<AddTwoInts::Request>();
  this->response_ = std::make_shared<AddTwoInts::Response>();
}

void call_server()
{
  auto result_future = client_->async_send_request(request_);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "service call failed :(");
  }
  this->response_=result_future.get();
}

std::shared_ptr<AddTwoInts::Request> request_;
std::shared_ptr<AddTwoInts::Response> response_;

private:
  rclcpp::Client<AddTwoInts>::SharedPtr client_;  
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalClient>();
  node->request_->a = 25;
  node->request_->b = 1;
  node->call_server();
  
  RCLCPP_INFO(
    node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
    node->request_->a, node->request_->b, node->response_->sum);
  
  rclcpp::shutdown();
  return 0;
}
