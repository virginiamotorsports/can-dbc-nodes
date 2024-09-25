// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file main.cpp
 * @brief Entry point for the ROS2.
 *
 * This file defines the main method that is run when ros2 is launched.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "msg_publisher/DbwNode.hpp"

/**
 * @brief Initializes and runs the CAN message publisher node.
 *
 * This function is run when ROS2 starts up. Initializes ROS2 and adds DbwNode to an executor.
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char ** argv)
{
  // init
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;

  // create and configure the node
  auto node = std::make_shared<msg_publisher::DbwNode>(options);  // dbw node
  exec.add_node(node->get_node_base_interface());                 // add node to executor

  // spin the executor to process callbacks (will block forever)
  exec.spin();

  // cleanup
  rclcpp::shutdown();
  return 0;
}
