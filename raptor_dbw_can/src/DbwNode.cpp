// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include "raptor_dbw_can/DbwNode.hpp"
#include <iostream>

#include <algorithm>
#include <cmath>
#include <string>

namespace raptor_dbw_can
{

DbwNode::DbwNode(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_can_node", options)
{
  dbcFile_ = this->declare_parameter("dbw_dbc_file", "");

  // Frame ID
  frame_id_ = "base_footprint";
  this->declare_parameter<std::string>("frame_id", frame_id_);
  

  // Set up Publishers
  pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 20);
  // pub_accel_pedal_ = this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>(
  //   "accelerator_pedal_report", 20);

  // autoware auto msg

  // pub_kinematic_state_ = this->create_publisher<autoware_auto_msgs::msg::VehicleKinematicState>("vehicle_kinematic_state", 10);

  // pub_pos_time_ = this->create_publisher<deep_orange_msgs::msg::PosTime>("pos_time", 2);

  sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_rx", 500, std::bind(&DbwNode::recvCAN, this, std::placeholders::_1));

  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFile_);


}

DbwNode::~DbwNode()
{
}

void DbwNode::recvCAN(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {
      case ID_WHEEL_SPEED_REPORT_DO:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_WHEEL_SPEED_REPORT_DO);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // raptor_dbw_msgs::msg::WheelSpeedReport out;
            // out.header.stamp = msg->header.stamp;

            // out.front_left  = message->GetSignal("wheel_speed_FL")->GetResult();
            // out.front_right = message->GetSignal("wheel_speed_FR")->GetResult();
            // out.rear_left = message->GetSignal("wheel_speed_RL")->GetResult();
            // out.rear_right = message->GetSignal("wheel_speed_RR")->GetResult();

            // pub_wheel_speeds_->publish(out);
          }
        }
        break;
    }
  }
}
}  // namespace raptor_dbw_can
