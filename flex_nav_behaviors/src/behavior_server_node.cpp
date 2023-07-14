// Copyright (c) 2022
//    Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
//     Christopher Newport University
// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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
// limitations under the License. Reserved.

#include <memory>
#include <string>

#include "flex_nav_behaviors/behavior_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto behaviors_node = std::make_shared<flex_nav_behavior_server::BehaviorServer>();

  rclcpp::spin(behaviors_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
