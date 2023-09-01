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

#include "raptor_dbw_can/CAN2ROS.hpp"
#include <iostream>

#include <algorithm>
#include <cmath>
#include <string>

namespace raptor_dbw_can
{

CAN2Node::CAN2Node(const rclcpp::NodeOptions & options)
: Node("can2ros", options)
{
  dbcFile_ = this->declare_parameter("dbw_dbc_file", "");

  // Frame ID
  frame_id_ = "base_footprint";
  this->declare_parameter<std::string>("frame_id", frame_id_);

  // Set up Publishers
  pub_motec_report = this->create_publisher<dash_msgs::msg::MotecReport>("motec_report", 10);


  sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
    "can1_rx", 500, std::bind(&CAN2Node::recvCAN, this, std::placeholders::_1));

  count_ = 0;

  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFile_);
  motec_timer = this->create_wall_timer(10ms, std::bind(&CAN2Node::motecPublisher, this));
}

CAN2Node::~CAN2Node()
{
}

void CAN2Node::motecPublisher(){
  pub_motec_report->publish(motec_report_msg);
}


void CAN2Node::recvCAN(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {
      
      case MOTEC_REPORT_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(MOTEC_REPORT_1);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg); 

            motec_report_msg.battery_voltage = message->GetSignal("BATTERY_VOLTAGE")->GetResult();
            motec_report_msg.fuel_pressure = message->GetSignal("FUEL_PRESSURE")->GetResult();
            motec_report_msg.coolant_temp = message->GetSignal("COOLANT_TEMP")->GetResult();
            motec_report_msg.oil_pressure = message->GetSignal("OIL_PRESSURE")->GetResult();

          }
        }
        break; 

      case MOTEC_REPORT_2:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(MOTEC_REPORT_2);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg); 

            motec_report_msg.oil_temp = message->GetSignal("OIL_TEMP")->GetResult();
            motec_report_msg.engine_rpm = message->GetSignal("ENGINE_RPM")->GetResult();

          }
        }
        break; 

      case MOTEC_REPORT_3:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(MOTEC_REPORT_3);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg); 

            motec_report_msg.throttle_position = message->GetSignal("THROTTLE_POS")->GetResult();
            motec_report_msg.front_brake_pressure = message->GetSignal("FRONT_BRAKE_PRESSURE")->GetResult();
            motec_report_msg.rear_brake_pressure = message->GetSignal("REAR_BRAKE_PRESSURE")->GetResult();
            motec_report_msg.ecu_temp = message->GetSignal("ECU_TEMP")->GetResult();

          }
        }
        break; 

      case MOTEC_REPORT_4:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(MOTEC_REPORT_4);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg); 

            motec_report_msg.map_sensor = message->GetSignal("MAP_SENSOR")->GetResult();
            motec_report_msg.intake_air_temp = message->GetSignal("INTAKE_AIR_TEMP")->GetResult();
            motec_report_msg.gear = message->GetSignal("GEAR")->GetResult();
            motec_report_msg.wheel_speed = message->GetSignal("WHEEL_SPEED")->GetResult();

          }
        }
        break; 

    }
  }
}







}  // namespace raptor_dbw_can
