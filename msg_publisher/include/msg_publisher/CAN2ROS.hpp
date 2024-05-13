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

#ifndef msg_publisher__CAN2Node_HPP_
#define msg_publisher__CAN2Node_HPP_

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <dash_msgs/msg/motec_report.hpp>


#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "msg_publisher/dispatch.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace msg_publisher
{
class CAN2Node : public rclcpp::Node
{
public:
  explicit CAN2Node(const rclcpp::NodeOptions & options);
  ~CAN2Node();

private:
  void recvCAN(const can_msgs::msg::Frame::SharedPtr msg);
  void motecPublisher();

  rclcpp::TimerBase::SharedPtr motec_timer;

  dash_msgs::msg::MotecReport motec_report_msg;

  // Frame ID
  std::string frame_id_;


  // Subscribed topics
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;

  // Published topics
  rclcpp::Publisher<dash_msgs::msg::MotecReport>::SharedPtr pub_motec_report;


  NewEagle::Dbc dbwDbc_;
  std::string dbcFile_;
  uint32_t count_;
};

}  // namespace msg_publisher

#endif  // msg_publisher__CAN2Node_HPP_
