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

#include "msg_publisher/DbwNode.hpp"
// #include "rclcpp_components/register_node_macro.hpp"
#include <iostream>

#include <algorithm>
#include <cmath>
#include <string>

namespace msg_publisher
{

DbwNode::DbwNode(const rclcpp::NodeOptions & options)
: Node("msg_publisher_node", options)
{
  dbcFile_can0_ = this->declare_parameter("dbw_dbc_file_can0", "");
  dbcFile_can1_ = this->declare_parameter("dbw_dbc_file_can1", "");
  dbcFile_can2_ = this->declare_parameter("dbw_dbc_file_can2", "");
  // Initializing tire report 
  // for (int i = 0; i < 16; i++) {
  //   tire_report_msg.fl_tire_temperature.push_back(0.0);
  //   tire_report_msg.fr_tire_temperature.push_back(0.0);
  //   tire_report_msg.rl_tire_temperature.push_back(0.0);
  //   tire_report_msg.rr_tire_temperature.push_back(0.0);
  // }

  // Set up Publishers
  pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 20);
  // pub_accel_pedal_ = this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>("accelerator_pedal_report", 20);
  pub_ams_report_ = this->create_publisher<vm_msgs::msg::AmsReport>("ams_report", 10);
  pub_brake_report_ = this->create_publisher<vm_msgs::msg::BrakeReport>("brake_report", 10);
  pub_dash_report_ = this->create_publisher<vm_msgs::msg::DashReport>("dash_report", 10);
  pub_suspension_report_ = this->create_publisher<vm_msgs::msg::SuspensionReport>("suspension_report", 10);
  pub_vcu_report_ = this->create_publisher<vm_msgs::msg::VcuReport>("vcu_report", 10);
  pub_inverter_report_ = this->create_publisher<vm_msgs::msg::InverterReport>("inverter_report", 10);


  // Set up Subscribers
  sub_can0_ = this->create_subscription<can_msgs::msg::Frame>(
    "can0_rx", 500, std::bind(&DbwNode::recvCAN0, this, std::placeholders::_1));
  // sub_can1_ = this->create_subscription<can_msgs::msg::Frame>(
  //   "can1_rx", 500, std::bind(&DbwNode::recvCAN1, this, std::placeholders::_1));
  // sub_can2_ = this->create_subscription<can_msgs::msg::Frame>(
  //   "can2_rx", 500, std::bind(&DbwNode::recvCAN2, this, std::placeholders::_1));
  // sub_brake_ = this->create_subscription<raptor_dbw_msgs::msg::BrakeCmd>(
  //   "brake_cmd", 1, std::bind(&DbwNode::recvBrakeCmd, this, std::placeholders::_1));

  dbwDbc_can0_ = NewEagle::DbcBuilder().NewDbc(dbcFile_can0_);
  // dbwDbc_can1_ = NewEagle::DbcBuilder().NewDbc(dbcFile_can1_);
  // dbwDbc_can2_ = NewEagle::DbcBuilder().NewDbc(dbcFile_can2_);

  timer_ams_report_ = this->create_wall_timer(
    10ms, std::bind(&DbwNode::timerAMSCallback, this));
  timer_brake_report_ = this->create_wall_timer(
    10ms, std::bind(&DbwNode::timerBrakeCallback, this));
  timer_dash_report_ = this->create_wall_timer(
    10ms, std::bind(&DbwNode::timerDashCallback, this));
  timer_suspension_report_ = this->create_wall_timer(
    10ms, std::bind(&DbwNode::timerSuspensionCallback, this));
  timer_vcu_report_ = this->create_wall_timer(
    10ms, std::bind(&DbwNode::timerVCUCallback, this));
  timer_inverter_report_ = this->create_wall_timer(
    10ms, std::bind(&DbwNode::timerInverterCallback, this));
}

DbwNode::~DbwNode()
{
}

void DbwNode::recvCAN0(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {
      // case ID_WHEEL_SPEED_REPORT_DO:
      //   {
      //    NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_WHEEL_SPEED_REPORT_DO);
      //     if (msg->dlc >= message->GetDlc()) {

      //       message->SetFrame(msg);

      //       raptor_dbw_msgs::msg::WheelSpeedReport out;
      //       out.header.stamp = msg->header.stamp;

      //       out.front_left  = message->GetSignal("wheel_speed_FL")->GetResult();
      //       out.front_right = message->GetSignal("wheel_speed_FR")->GetResult();
      //       out.rear_left = message->GetSignal("wheel_speed_RL")->GetResult();
      //       out.rear_right = message->GetSignal("wheel_speed_RR")->GetResult();

      //       pub_wheel_speeds_->publish(out);
      //     }
      //   }
      //   break;

    //   case ID_BRAKE_PRESSURE_REPORT_DO:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_BRAKE_PRESSURE_REPORT_DO);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);

    //         raptor_dbw_msgs::msg::Brake2Report out;
    //         out.header.stamp = msg->header.stamp;


    //         out.front_brake_pressure  = message->GetSignal("brake_pressure_fdbk_front")->GetResult();
    //         out.rear_brake_pressure  = message->GetSignal("brake_pressure_fdbk_rear")->GetResult();
    //         out.rolling_counter = message->GetSignal("brk_pressure_fdbk_counter")->GetResult();
            
    //         pub_brake_2_report_->publish(out);
    //       }
    //     }
    //     break;

    //   case ID_ACCELERATOR_REPORT_DO:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_ACCELERATOR_REPORT_DO);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);

    //         raptor_dbw_msgs::msg::AcceleratorPedalReport out;
    //         out.header.stamp = msg->header.stamp;

    //         out.pedal_output  = message->GetSignal("acc_pedal_fdbk")->GetResult();
    //         out.rolling_counter = message->GetSignal("acc_pedal_fdbk_counter")->GetResult(); 
    //         pub_accel_pedal_->publish(out);
    //       }
    //     }
    //     break;

    //   case ID_STEERING_REPORT_DO:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_STEERING_REPORT_DO);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);
            

    //         raptor_dbw_msgs::msg::SteeringReport out;
    //         out.header.stamp = msg->header.stamp;

    //         out.steering_wheel_angle  = message->GetSignal("steering_motor_ang_avg_fdbk")->GetResult();
    //         out.rolling_counter = message->GetSignal("steering_motor_fdbk_counter")->GetResult(); 
    //         pub_steering_->publish(out);
    //       }
    //     }
    //     break;

    //   case ID_STEERING_REPORT_EXTD:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_STEERING_REPORT_EXTD);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);
    //         raptor_dbw_msgs::msg::SteeringExtendedReport out;
    //         out.header.stamp = msg->header.stamp;

    //         out.steering_motor_ang_1  = message->GetSignal("steering_motor_ang_1_fdbk")->GetResult();
    //         out.steering_motor_ang_2  = message->GetSignal("steering_motor_ang_2_fdbk")->GetResult();
    //         out.steering_motor_ang_3  = message->GetSignal("steering_motor_ang_3_fdbk")->GetResult();
    //         pub_steering_ext_->publish(out);
    //       }
    //     }
    //     break;

    //   case ID_MISC_REPORT_DO:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_MISC_REPORT_DO);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);

    //         deep_orange_msgs::msg::MiscReport out;

    //         // out.off_grid_power_connection  = message->GetSignal("off_grid_power_connection")->GetResult();
    //         out.stamp = msg->header.stamp;
    //         out.sys_state = message->GetSignal("sys_state")->GetResult(); 
    //         out.safety_switch_state = message->GetSignal("safety_switch_state")->GetResult(); 
    //         out.mode_switch_state = message->GetSignal("mode_switch_state")->GetResult();
    //         out.battery_voltage = message->GetSignal("battery_voltage")->GetResult();
    //         pub_misc_do_->publish(out);
    //       }
    //     }
    //     break;

    //   case ID_RC_TO_CT:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_RC_TO_CT);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);

    //         deep_orange_msgs::msg::RcToCt out;
    //         out.stamp = msg->header.stamp;
    //         // out.current_position  = message->GetSignal("DBW_CurrentPosition")->GetResult();
    //         out.track_flag = message->GetSignal("track_flag")->GetResult(); 
    //         out.veh_flag = message->GetSignal("veh_flag")->GetResult();
    //         out.lap_count = message->GetSignal("lap_count")->GetResult();
    //         out.lap_distance = message->GetSignal("lap_distance")->GetResult();
    //         out.target_speed = message->GetSignal("round_target_speed")->GetResult();
    //         out.rolling_counter = message->GetSignal("base_to_car_heartbeat")->GetResult();
    //         pub_rc_to_ct_->publish(out);
    //       }
    //     }
    //     break;

    //   case ID_BASE_TO_CAR_TIMING:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_BASE_TO_CAR_TIMING);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);

    //         deep_orange_msgs::msg::BaseToCarTiming out;
    //         out.stamp = msg->header.stamp;
    //         // out.current_position  = message->GetSignal("DBW_CurrentPosition")->GetResult();
    //         out.laps = message->GetSignal("laps")->GetResult(); 
    //         out.lap_time = message->GetSignal("lap_time")->GetResult(); 
    //         out.time_stamp = message->GetSignal("time_stamp")->GetResult(); 

    //         pub_base_to_car_timing_->publish(out);
    //       }
    //     }
    //     break;

    //   case ID_PT_REPORT_1:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_PT_REPORT_1);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);
    //         pt_report_msg.stamp = msg->header.stamp;
    //         // pt_report_msg.sys_state = message->GetSignal("engine_state")->GetResult(); 
    //         // pt_report_msg.safety_switch_state = message->GetSignal("engine_run_switch")->GetResult(); 
    //         pt_report_msg.throttle_position = message->GetSignal("throttle_position")->GetResult();
    //         pt_report_msg.engine_run_switch_status = message->GetSignal("engine_run_switch")->GetResult();
    //         pt_report_msg.current_gear = message->GetSignal("current_gear")->GetResult();
    //         pt_report_msg.engine_rpm = message->GetSignal("engine_speed_rpm")->GetResult();
    //         pt_report_msg.vehicle_speed_kmph = message->GetSignal("vehicle_speed_kmph")->GetResult();
    //       }
    //     }
    //     break;

    //   case ID_PT_REPORT_2:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_PT_REPORT_2);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);

    //         pt_report_msg.fuel_pressure = message->GetSignal("fuel_pressure_kPa")->GetResult();
    //         pt_report_msg.engine_oil_pressure = message->GetSignal("engine_oil_pressure_kPa")->GetResult();
    //         pt_report_msg.engine_coolant_temperature = message->GetSignal("coolant_temperature")->GetResult();
    //         pt_report_msg.transmission_oil_temperature = message->GetSignal("transmission_temperature")->GetResult();
    //         pt_report_msg.transmission_oil_pressure = message->GetSignal("transmission_pressure_kPa")->GetResult();
    //       }
    //     }
    //     break;

    //   case ID_TIRE_PRESSURE_FL:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_PRESSURE_FL);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);
    //         tire_report_msg.fl_tire_pressure = message->GetSignal("FL_Tire_Pressure")->GetResult();
    //         tire_report_msg.fl_tire_pressure_gauge = message->GetSignal("FL_Tire_Pressure_Gauge")->GetResult();
    //       }
    //     }
    //     break;

    //   case ID_TIRE_PRESSURE_FR:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_PRESSURE_FR);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);
    //         tire_report_msg.fr_tire_pressure = message->GetSignal("FR_Tire_Pressure")->GetResult();
    //         tire_report_msg.fr_tire_pressure_gauge = message->GetSignal("FR_Tire_Pressure_Gauge")->GetResult();
    //       }
    //     }
    //     break;

    //   case ID_TIRE_PRESSURE_RL:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_PRESSURE_RL);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);
    //         tire_report_msg.rl_tire_pressure = message->GetSignal("RL_Tire_Pressure")->GetResult();
    //         tire_report_msg.rl_tire_pressure_gauge = message->GetSignal("RL_Tire_Pressure_Gauge")->GetResult();
    //       }
    //     }
    //     break;
      
    //   case ID_TIRE_PRESSURE_RR:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_PRESSURE_RR);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);
    //         tire_report_msg.rr_tire_pressure = message->GetSignal("RR_Tire_Pressure")->GetResult();
    //         tire_report_msg.rr_tire_pressure_gauge = message->GetSignal("RR_Tire_Pressure_Gauge")->GetResult();
    //       }
    //     }
    //     break;

    //   case ID_TIRE_TEMP_FL_1:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_TIRE_TEMP_FL_1);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);
    //         tire_report_msg.fl_tire_temperature[0] = message->GetSignal("FL_Tire_Temp_01")->GetResult();
    //         tire_report_msg.fl_tire_temperature[1] = message->GetSignal("FL_Tire_Temp_02")->GetResult();
    //         tire_report_msg.fl_tire_temperature[2] = message->GetSignal("FL_Tire_Temp_03")->GetResult();
    //         tire_report_msg.fl_tire_temperature[3] = message->GetSignal("FL_Tire_Temp_04")->GetResult();

    //       }
    //     }
    //     break;
      
    //   case ID_WHEEL_STRAIN_GAUGE:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_WHEEL_STRAIN_GAUGE);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg); 
    //         tire_report_msg.fl_wheel_load = message->GetSignal("wheel_strain_gauge_FL")->GetResult();
    //         tire_report_msg.fr_wheel_load = message->GetSignal("wheel_strain_gauge_FR")->GetResult();
    //         tire_report_msg.rl_wheel_load = message->GetSignal("wheel_strain_gauge_RL")->GetResult();
    //         tire_report_msg.rr_wheel_load = message->GetSignal("wheel_strain_gauge_RR")->GetResult();

    //       }
    //     }
    //     break; 

    //   case ID_PT_REPORT_3:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_PT_REPORT_3);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg); 

    //         pt_report_msg.engine_oil_temperature = message->GetSignal("engine_oil_temperature")->GetResult();
    //         pt_report_msg.torque_wheels = message->GetSignal("torque_wheels")->GetResult();

    //       }
    //     }
    //     break; 

    //   case ID_WHEEL_POTENTIOMETER:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_WHEEL_POTENTIOMETER);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg); 
    //         tire_report_msg.stamp = msg->header.stamp;
    //         tire_report_msg.fl_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_FL")->GetResult();
    //         tire_report_msg.fr_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_FR")->GetResult();
    //         tire_report_msg.rl_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_RL")->GetResult();
    //         tire_report_msg.rr_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_RR")->GetResult();

    //       }
    //     }
    //     break;  

      
    //   case ID_DIAGNOSTIC_REPORT:
    //     {
    //      NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(ID_DIAGNOSTIC_REPORT);
    //       if (msg->dlc >= message->GetDlc()) {

    //         message->SetFrame(msg);

    //         raptor_dbw_msgs::msg::DiagnosticReport out;
    //         out.header = msg->header;
    //         out.sd_system_warning = message->GetSignal("sd_system_warning")->GetResult(); 
    //         out.sd_system_failure = message->GetSignal("sd_system_failure")->GetResult(); 
    //         out.sd_brake_warning1 = message->GetSignal("sd_brake_warning1")->GetResult(); 
    //         out.sd_brake_warning2 = message->GetSignal("sd_brake_warning2")->GetResult(); 
    //         out.sd_brake_warning3 = message->GetSignal("sd_brake_warning3")->GetResult(); 
    //         out.sd_steer_warning1 = message->GetSignal("sd_steer_warning1")->GetResult(); 
    //         out.sd_steer_warning2 = message->GetSignal("sd_steer_warning2")->GetResult(); 
    //         out.sd_steer_warning3 = message->GetSignal("sd_steer_warning3")->GetResult(); 
    //         out.motec_warning = message->GetSignal("motec_warning")->GetResult(); 
    //         out.front_brk_failure = message->GetSignal("est1_oos_front_brk")->GetResult(); 
    //         out.rear_brk_failure = message->GetSignal("est2_oos_rear_brk")->GetResult(); 
    //         out.low_eng_speed = message->GetSignal("est3_low_eng_speed")->GetResult(); 
    //         out.sd_comms_loss = message->GetSignal("est4_sd_comms_loss")->GetResult(); 
    //         out.motec_comms_loss = message->GetSignal("est5_motec_comms_loss")->GetResult(); 
    //         out.sd_ebrake = message->GetSignal("est6_sd_ebrake")->GetResult(); 
    //         out.adlink_hb_lost = message->GetSignal("adlink_hb_lost")->GetResult(); 
    //         out.rc_lost = message->GetSignal("rc_lost")->GetResult(); 

    //         pub_diagnostic_report_->publish(out);
    //       }
    //     }
    //     break;
    }
  }
}

  // void DbwNode::recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg)
  // {
  //   NewEagle::DbcMessage * message = dbwDbc_can0_.GetMessage("brake_pressure_cmd"); 
  //   message->GetSignal("brake_pressure_cmd")->SetResult(msg->pedal_cmd); 
  //   message->GetSignal("brk_pressure_cmd_counter")->SetResult(msg->rolling_counter);

  //   can_msgs::msg::Frame frame = message->GetFrame();

  //   pub_can_->publish(frame);
  // }

  void DbwNode::timerBrakeCallback() {
      pub_brake_report_->publish(brake_report_msg_);
  }
  void DbwNode::timerAMSCallback() {
      pub_ams_report_->publish(ams_report_msg_);
  }
  void DbwNode::timerVCUCallback() {
      pub_vcu_report_->publish(vcu_report_msg_);
  }
  void DbwNode::timerDashCallback() {
      pub_dash_report_->publish(dash_report_msg_);
  }
  void DbwNode::timerSuspensionCallback() {
      pub_suspension_report_->publish(suspension_report_msg_);
  }
  void DbwNode::timerInverterCallback() {
      pub_inverter_report_->publish(inverter_report_msg_);
  }
  

}  // namespace msg_publisher

// RCLCPP_COMPONENTS_REGISTER_NODE(msg_publisher::DbwNode)