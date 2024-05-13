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

#ifndef msg_publisher__DBWNODE_HPP_
#define msg_publisher__DBWNODE_HPP_

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
// #include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>

#include <vm_msgs/msg/ams_report.hpp>
#include <vm_msgs/msg/brake_report.hpp>
#include <vm_msgs/msg/dash_report.hpp>
#include <vm_msgs/msg/suspension_report.hpp>
#include <vm_msgs/msg/vcu_report.hpp>
#include <vm_msgs/msg/inverter_report.hpp>


#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
class DbwNode : public rclcpp::Node
{
public:
  explicit DbwNode(const rclcpp::NodeOptions & options);
  ~DbwNode();

private:

  void timerAMSCallback();
  void timerVCUCallback();
  void timerDashCallback();
  void timerSuspensionCallback();
  void timerInverterCallback();
  void timerBrakeCallback();

  void recvCAN0(const can_msgs::msg::Frame::SharedPtr msg);
  void recvCAN1(const can_msgs::msg::Frame::SharedPtr msg);
  void recvCAN2(const can_msgs::msg::Frame::SharedPtr msg);
  // void recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg);

  vm_msgs::msg::AmsReport ams_report_msg_;
  vm_msgs::msg::BrakeReport brake_report_msg_;
  vm_msgs::msg::DashReport dash_report_msg_;
  vm_msgs::msg::SuspensionReport suspension_report_msg_;
  vm_msgs::msg::VcuReport vcu_report_msg_;
  vm_msgs::msg::InverterReport inverter_report_msg_;


  rclcpp::TimerBase::SharedPtr timer_ams_report_;
  rclcpp::TimerBase::SharedPtr timer_vcu_report_;
  rclcpp::TimerBase::SharedPtr timer_inverter_report_;
  rclcpp::TimerBase::SharedPtr timer_dash_report_;
  rclcpp::TimerBase::SharedPtr timer_brake_report_;
  rclcpp::TimerBase::SharedPtr timer_suspension_report_;

  // Subscribed topics
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can0_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can1_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can2_;
  // rclcpp::Subscription<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr sub_brake_;


  // Published topics
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  // rclcpp::Publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>::SharedPtr pub_accel_pedal_; // acc pedal report do
  rclcpp::Publisher<vm_msgs::msg::AmsReport>::SharedPtr pub_ams_report_;
  rclcpp::Publisher<vm_msgs::msg::BrakeReport>::SharedPtr pub_brake_report_; 
  rclcpp::Publisher<vm_msgs::msg::DashReport>::SharedPtr pub_dash_report_; 
  rclcpp::Publisher<vm_msgs::msg::SuspensionReport>::SharedPtr pub_suspension_report_; 
  rclcpp::Publisher<vm_msgs::msg::VcuReport>::SharedPtr pub_vcu_report_; 
  rclcpp::Publisher<vm_msgs::msg::InverterReport>::SharedPtr pub_inverter_report_; 
  

  NewEagle::Dbc dbwDbc_can0_;
  NewEagle::Dbc dbwDbc_can1_;
  NewEagle::Dbc dbwDbc_can2_;
  std::string dbcFile_can0_;
  std::string dbcFile_can1_;
  std::string dbcFile_can2_;

};

}  // namespace msg_publisher

#endif  // msg_publisher__DBWNODE_HPP_
