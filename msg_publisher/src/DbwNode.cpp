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

      case MODULATION_AND_FLUX_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Iq_Command = message->GetSignal("INV_Iq_Command")->GetResult();
                inverter_report_msg_.INV_Id_Command = message->GetSignal("INV_Id_Command")->GetResult();
                inverter_report_msg_.INV_Flux_Weakening_Output = message->GetSignal("INV_Flux_Weakening_Output")->GetResult();
                inverter_report_msg_.INV_Modulation_Index = message->GetSignal("INV_Modulation_Index")->GetResult();
            }
        }
        break;

      case TORQUE_AND_TIMER_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Power_On_Timer = message->GetSignal("INV_Power_On_Timer")->GetResult();
                inverter_report_msg_.INV_Torque_Feedback = message->GetSignal("INV_Torque_Feedback")->GetResult();
                inverter_report_msg_.INV_Commanded_Torque = message->GetSignal("INV_Commanded_Torque")->GetResult();
            }
        }
        break;

      case READ_WRITE_PARAM_RESPONSE:
        {
          // NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
          //   if (msg->dlc >= message->GetDlc()) {
          //       message->SetFrame(msg);

          //       inverter_report_msg_.INV_Parameter_Response_Write_OK = message->GetSignal("INV_Parameter_Response_Write_OK")->GetResult();
          //       inverter_report_msg_.INV_Parameter_Response_Data = message->GetSignal("INV_Parameter_Response_Data")->GetResult();
          //       inverter_report_msg_.INV_Parameter_Response_Addr = message->GetSignal("INV_Parameter_Response_Addr")->GetResult();
          //   }
        }
        break;

      case READ_WRITE_PARAM_COMMAND:
        {
          // NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
          //   if (msg->dlc >= message->GetDlc()) {
          //       message->SetFrame(msg);

          //       inverter_report_msg_.VCU_INV_Parameter_Data = message->GetSignal("VCU_INV_Parameter_Data")->GetResult();
          //       inverter_report_msg_.VCU_INV_Parameter_RW_Command = message->GetSignal("VCU_INV_Parameter_RW_Command")->GetResult();
          //       inverter_report_msg_.VCU_INV_Parameter_Address = message->GetSignal("VCU_INV_Parameter_Address")->GetResult();
          //   }
        }
        break;

      case COMMAND_MESSAGE:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.VCU_INV_Inverter_Enable = message->GetSignal("VCU_INV_Inverter_Enable")->GetResult();
                inverter_report_msg_.VCU_INV_Direction_Command = message->GetSignal("VCU_INV_Direction_Command")->GetResult();
                inverter_report_msg_.VCU_INV_Speed_Command = message->GetSignal("VCU_INV_Speed_Command")->GetResult();
                inverter_report_msg_.VCU_INV_Torque_Command = message->GetSignal("VCU_INV_Torque_Command")->GetResult();
                inverter_report_msg_.VCU_INV_Inverter_Discharge = message->GetSignal("VCU_INV_Inverter_Discharge")->GetResult();
                inverter_report_msg_.VCU_INV_Torque_Limit_Command = message->GetSignal("VCU_INV_Torque_Limit_Command")->GetResult();
                inverter_report_msg_.VCU_INV_Speed_Mode_Enable = message->GetSignal("VCU_INV_Speed_Mode_Enable")->GetResult();
                inverter_report_msg_.VCU_INV_Rolling_Counter = message->GetSignal("VCU_INV_Rolling_Counter")->GetResult();
            }
        }
        break;

      case FAULT_CODES:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Run_Fault_Hi = message->GetSignal("INV_Run_Fault_Hi")->GetResult();
                inverter_report_msg_.INV_Post_Fault_Hi = message->GetSignal("INV_Post_Fault_Hi")->GetResult();
                inverter_report_msg_.INV_Run_Fault_Lo = message->GetSignal("INV_Run_Fault_Lo")->GetResult();
                inverter_report_msg_.INV_Post_Fault_Lo = message->GetSignal("INV_Post_Fault_Lo")->GetResult();
            }
        }
        break;

      case INTERAL_STATES:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Direction_Command = message->GetSignal("INV_Direction_Command")->GetResult();
                inverter_report_msg_.INV_Inverter_Enable_State = message->GetSignal("INV_Inverter_Enable_State")->GetResult();
                inverter_report_msg_.INV_Relay_3_Status = message->GetSignal("INV_Relay_3_Status")->GetResult();
                inverter_report_msg_.INV_Relay_4_Status = message->GetSignal("INV_Relay_4_Status")->GetResult();
                inverter_report_msg_.INV_Relay_2_Status = message->GetSignal("INV_Relay_2_Status")->GetResult();
                inverter_report_msg_.INV_Inverter_Run_Mode = message->GetSignal("INV_Inverter_Run_Mode")->GetResult();
                inverter_report_msg_.INV_Inverter_Command_Mode = message->GetSignal("INV_Inverter_Command_Mode")->GetResult();
                inverter_report_msg_.INV_Relay_1_Status = message->GetSignal("INV_Relay_1_Status")->GetResult();
                inverter_report_msg_.INV_Inverter_State = message->GetSignal("INV_Inverter_State")->GetResult();
                inverter_report_msg_.INV_VSM_State = message->GetSignal("INV_VSM_State")->GetResult();
                inverter_report_msg_.INV_Inverter_Enable_Lockout = message->GetSignal("INV_Inverter_Enable_Lockout")->GetResult();
                inverter_report_msg_.INV_Inverter_Discharge_State = message->GetSignal("INV_Inverter_Discharge_State")->GetResult();
                inverter_report_msg_.INV_Relay_5_Status = message->GetSignal("INV_Relay_5_Status")->GetResult();
                inverter_report_msg_.INV_Relay_6_Status = message->GetSignal("INV_Relay_6_Status")->GetResult();
                inverter_report_msg_.INV_BMS_Active = message->GetSignal("INV_BMS_Active")->GetResult();
                inverter_report_msg_.INV_BMS_Torque_Limiting = message->GetSignal("INV_BMS_Torque_Limiting")->GetResult();
                inverter_report_msg_.INV_PWM_Frequency = message->GetSignal("INV_PWM_Frequency")->GetResult();
                inverter_report_msg_.INV_Limit_Max_Speed = message->GetSignal("INV_Limit_Max_Speed")->GetResult();
                inverter_report_msg_.INV_Limit_Hot_Spot = message->GetSignal("INV_Limit_Hot_Spot")->GetResult();
                inverter_report_msg_.INV_Low_Speed_Limiting = message->GetSignal("INV_Low_Speed_Limiting")->GetResult();
                inverter_report_msg_.INV_Rolling_Counter = message->GetSignal("INV_Rolling_Counter")->GetResult();
                inverter_report_msg_.INV_Limit_Coolant_Derating = message->GetSignal("INV_Limit_Coolant_Derating")->GetResult();
                inverter_report_msg_.INV_Self_Sensing_Assist_Enable = message->GetSignal("INV_Self_Sensing_Assist_Enable")->GetResult();
                inverter_report_msg_.INV_Limit_Stall_Burst_Model = message->GetSignal("INV_Limit_Stall_Burst_Model")->GetResult();
                inverter_report_msg_.INV_Burst_Model_Mode = message->GetSignal("INV_Burst_Model_Mode")->GetResult();
                inverter_report_msg_.INV_Key_Switch_Start_Status = message->GetSignal("INV_Key_Switch_Start_Status")->GetResult();
            }
        }
        break;

      case INTERNAL_VOLTAGES:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Ref_Voltage_12_0 = message->GetSignal("INV_Ref_Voltage_12_0")->GetResult();
                inverter_report_msg_.INV_Ref_Voltage_5_0 = message->GetSignal("INV_Ref_Voltage_5_0")->GetResult();
                inverter_report_msg_.INV_Ref_Voltage_2_5 = message->GetSignal("INV_Ref_Voltage_2_5")->GetResult();
                inverter_report_msg_.INV_Ref_Voltage_1_5 = message->GetSignal("INV_Ref_Voltage_1_5")->GetResult();
            }
        }
        break;

      case FLUX_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Iq = message->GetSignal("INV_Iq")->GetResult();
                inverter_report_msg_.INV_Id = message->GetSignal("INV_Id")->GetResult();
                inverter_report_msg_.INV_Vq_ff = message->GetSignal("INV_Vq_ff")->GetResult();
                inverter_report_msg_.INV_Vd_ff = message->GetSignal("INV_Vd_ff")->GetResult();
            }
        }
        break;

      case VOLTAGE_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_VBC_Vq_Voltage = message->GetSignal("INV_VBC_Vq_Voltage")->GetResult();
                inverter_report_msg_.INV_VAB_Vd_Voltage = message->GetSignal("INV_VAB_Vd_Voltage")->GetResult();
                inverter_report_msg_.INV_Output_Voltage = message->GetSignal("INV_Output_Voltage")->GetResult();
                inverter_report_msg_.INV_DC_Bus_Voltage = message->GetSignal("INV_DC_Bus_Voltage")->GetResult();
            }
        }
        break;

      case CURRENT_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_DC_Bus_Current = message->GetSignal("INV_DC_Bus_Current")->GetResult();
                inverter_report_msg_.INV_Phase_C_Current = message->GetSignal("INV_Phase_C_Current")->GetResult();
                inverter_report_msg_.INV_Phase_B_Current = message->GetSignal("INV_Phase_B_Current")->GetResult();
                inverter_report_msg_.INV_Phase_A_Current = message->GetSignal("INV_Phase_A_Current")->GetResult();
            }
        }
        break;

      case MOTOR_POSITION:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Delta_Resolver_Filtered = message->GetSignal("INV_Delta_Resolver_Filtered")->GetResult();
                inverter_report_msg_.INV_Electrical_Output_Frequency = message->GetSignal("INV_Electrical_Output_Frequency")->GetResult();
                inverter_report_msg_.INV_Motor_Speed = message->GetSignal("INV_Motor_Speed")->GetResult();
                inverter_report_msg_.INV_Motor_Angle_Electrical = message->GetSignal("INV_Motor_Angle_Electrical")->GetResult();
            }
        }
        break;

      case DIGITAL_INPUT_STATUS:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Digital_Input_5 = message->GetSignal("INV_Digital_Input_5")->GetResult();
                inverter_report_msg_.INV_Digital_Input_4 = message->GetSignal("INV_Digital_Input_4")->GetResult();
                inverter_report_msg_.INV_Digital_Input_3 = message->GetSignal("INV_Digital_Input_3")->GetResult();
                inverter_report_msg_.INV_Digital_Input_2 = message->GetSignal("INV_Digital_Input_2")->GetResult();
                inverter_report_msg_.INV_Digital_Input_1 = message->GetSignal("INV_Digital_Input_1")->GetResult();
                inverter_report_msg_.INV_Digital_Input_6 = message->GetSignal("INV_Digital_Input_6")->GetResult();
                inverter_report_msg_.INV_Digital_Input_7 = message->GetSignal("INV_Digital_Input_7")->GetResult();
                inverter_report_msg_.INV_Digital_Input_8 = message->GetSignal("INV_Digital_Input_8")->GetResult();
            }
        }
        break;

      case ANALOG_INPUT_VOLTAGE:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Analog_Input_1 = message->GetSignal("INV_Analog_Input_1")->GetResult();
                inverter_report_msg_.INV_Analog_Input_2 = message->GetSignal("INV_Analog_Input_2")->GetResult();
                inverter_report_msg_.INV_Analog_Input_3 = message->GetSignal("INV_Analog_Input_3")->GetResult();
                inverter_report_msg_.INV_Analog_Input_4 = message->GetSignal("INV_Analog_Input_4")->GetResult();
                inverter_report_msg_.INV_Analog_Input_5 = message->GetSignal("INV_Analog_Input_5")->GetResult();
                inverter_report_msg_.INV_Analog_Input_6 = message->GetSignal("INV_Analog_Input_6")->GetResult();
            }
        }
        break;

      case TEMP_SET_3:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Torque_Shudder = message->GetSignal("INV_Torque_Shudder")->GetResult();
                inverter_report_msg_.INV_Motor_Temp = message->GetSignal("INV_Motor_Temp")->GetResult();
                inverter_report_msg_.INV_Hot_Spot_Temp = message->GetSignal("INV_Hot_Spot_Temp")->GetResult();
                inverter_report_msg_.INV_Coolant_Temp = message->GetSignal("INV_Coolant_Temp")->GetResult();
            }
        }
        break;

      case TEMP_SET_2:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_RTD2_Temperature = message->GetSignal("INV_RTD2_Temperature")->GetResult();
                inverter_report_msg_.INV_RTD1_Temperature = message->GetSignal("INV_RTD1_Temperature")->GetResult();
                inverter_report_msg_.INV_Control_Board_Temp = message->GetSignal("INV_Control_Board_Temp")->GetResult();
                inverter_report_msg_.INV_Stall_Burst_Model_Temp = message->GetSignal("INV_Stall_Burst_Model_Temp")->GetResult();
            }
        }
        break;

      case TEMP_SET_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Gate_Driver_Board_Temp = message->GetSignal("INV_Gate_Driver_Board_Temp")->GetResult();
                inverter_report_msg_.INV_Module_C_Temp = message->GetSignal("INV_Module_C_Temp")->GetResult();
                inverter_report_msg_.INV_Module_B_Temp = message->GetSignal("INV_Module_B_Temp")->GetResult();
                inverter_report_msg_.INV_Module_A_Temp = message->GetSignal("INV_Module_A_Temp")->GetResult();
            }
        }
        break;

      case FIRMWARE_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Project_Code_EEP_Ver = message->GetSignal("INV_Project_Code_EEP_Ver")->GetResult();
                inverter_report_msg_.INV_SW_Version = message->GetSignal("INV_SW_Version")->GetResult();
                inverter_report_msg_.INV_DateCode_MMDD = message->GetSignal("INV_DateCode_MMDD")->GetResult();
                inverter_report_msg_.INV_DateCode_YYYY = message->GetSignal("INV_DateCode_YYYY")->GetResult();
            }
        }
        break;

      case DIAGNOSTIC_DATA_MSGS:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Diag_Record = message->GetSignal("INV_Diag_Record")->GetResult();
                inverter_report_msg_.INV_Diag_Segment = message->GetSignal("INV_Diag_Segment")->GetResult();
                inverter_report_msg_.INV_Diag_Gamma_Resolver = message->GetSignal("INV_Diag_Gamma_Resolver")->GetResult();
                inverter_report_msg_.INV_Diag_Gamma_Observer = message->GetSignal("INV_Diag_Gamma_Observer")->GetResult();
                inverter_report_msg_.INV_Diag_Sin_Used = message->GetSignal("INV_Diag_Sin_Used")->GetResult();
                inverter_report_msg_.INV_Diag_Cos_Used = message->GetSignal("INV_Diag_Cos_Used")->GetResult();
                inverter_report_msg_.INV_Diag_Ia = message->GetSignal("INV_Diag_Ia")->GetResult();
                inverter_report_msg_.INV_Diag_Ib = message->GetSignal("INV_Diag_Ib")->GetResult();
                inverter_report_msg_.INV_Diag_Ic = message->GetSignal("INV_Diag_Ic")->GetResult();
                inverter_report_msg_.INV_Diag_Vdc = message->GetSignal("INV_Diag_Vdc")->GetResult();
                inverter_report_msg_.INV_Diag_Iq_cmd = message->GetSignal("INV_Diag_Iq_cmd")->GetResult();
                inverter_report_msg_.INV_Diag_Id_cmd = message->GetSignal("INV_Diag_Id_cmd")->GetResult();
                inverter_report_msg_.INV_Diag_Mod_Index = message->GetSignal("INV_Diag_Mod_Index")->GetResult();
                inverter_report_msg_.INV_Diag_FW_Output = message->GetSignal("INV_Diag_FW_Output")->GetResult();
                inverter_report_msg_.INV_Diag_Vq_Cmd = message->GetSignal("INV_Diag_Vq_Cmd")->GetResult();
                inverter_report_msg_.INV_Diag_Vd_Cmd = message->GetSignal("INV_Diag_Vd_Cmd")->GetResult();
                inverter_report_msg_.INV_Diag_Vqs_Cmd = message->GetSignal("INV_Diag_Vqs_Cmd")->GetResult();
                inverter_report_msg_.INV_Diag_PWM_Freq = message->GetSignal("INV_Diag_PWM_Freq")->GetResult();
                inverter_report_msg_.INV_Diag_Run_Faults_Lo = message->GetSignal("INV_Diag_Run_Faults_Lo")->GetResult();
                inverter_report_msg_.INV_Diag_Run_Faults_Hi = message->GetSignal("INV_Diag_Run_Faults_Hi")->GetResult();
            }
        }
        break;

      case FAST_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Fast_Torque_Command = message->GetSignal("INV_Fast_Torque_Command")->GetResult();
                inverter_report_msg_.INV_Fast_Torque_Feedback = message->GetSignal("INV_Fast_Torque_Feedback")->GetResult();
                inverter_report_msg_.INV_Fast_Motor_Speed = message->GetSignal("INV_Fast_Motor_Speed")->GetResult();
                inverter_report_msg_.INV_Fast_DC_Bus_Voltage = message->GetSignal("INV_Fast_DC_Bus_Voltage")->GetResult();
            }
        }
        break;

      case TORQUE_CAPABILITY:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.INV_Torque_Capability = message->GetSignal("INV_Torque_Capability")->GetResult();
            }
        }
        break;
        
      case CURRENT_LIMIT:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.BMS_Max_Discharge_Current = message->GetSignal("BMS_Max_Discharge_Current")->GetResult();
                inverter_report_msg_.BMS_Max_Charge_Current = message->GetSignal("BMS_Max_Charge_Current")->GetResult();
            }
        }
        break;
        
      // Add cases for other messages as needed
      case STATUS:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(256);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::Status out;
            // out.header.stamp = msg->header.stamp;
            // Populate the 'out' message with relevant signals here.
            // pub_status_->publish(out);
          }
        }
        break;

      case FAULT:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(257);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::Faults out;
            // out.header.stamp = msg->header.stamp;
            // Populate the 'out' message with relevant signals here.
            // pub_faults_->publish(out);
          }
        }
        break;

      case DASH_REPORT:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(258);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::Dashboard out;
            // out.header.stamp = msg->header.stamp;
            // Populate the 'out' message with relevant signals here.
            // pub_dashboard_->publish(out);
          }
        }
        break;

      case CELL_TEMP_0:  // Fall-through
      case CELL_TEMP_1:
      case CELL_TEMP_2:
      case CELL_TEMP_3:
      case CELL_TEMP_4:
      case CELL_TEMP_5:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            uint32_t index = (msg->id - CELL_TEMP_0) * 8;
            // vm_msgs::msg::VoltageSet out;
            ams_report_msg_.cell_temperatures[0 + index] = message->GetSignal("Temp" + ( 1 + index))->GetResult();
            ams_report_msg_.cell_temperatures[1 + index] = message->GetSignal("Temp" + ( 2 + index))->GetResult();
            ams_report_msg_.cell_temperatures[2 + index] = message->GetSignal("Temp" + ( 3 + index))->GetResult();
            ams_report_msg_.cell_temperatures[3 + index] = message->GetSignal("Temp" + ( 4 + index))->GetResult();
            ams_report_msg_.cell_temperatures[4 + index] = message->GetSignal("Temp" + ( 5 + index))->GetResult();
            ams_report_msg_.cell_temperatures[5 + index] = message->GetSignal("Temp" + ( 6 + index))->GetResult();
            ams_report_msg_.cell_temperatures[6 + index] = message->GetSignal("Temp" + ( 7 + index))->GetResult();
            ams_report_msg_.cell_temperatures[7 + index] = message->GetSignal("Temp" + ( 8 + index))->GetResult();
          }
        }
        break;
      case CELL_VOLTAGE_0:  // Fall-through
      case CELL_VOLTAGE_1:
      case CELL_VOLTAGE_2:
      case CELL_VOLTAGE_3:
      case CELL_VOLTAGE_4:
      case CELL_VOLTAGE_5:
      case CELL_VOLTAGE_6:
      case CELL_VOLTAGE_7:
      case CELL_VOLTAGE_8:
      case CELL_VOLTAGE_9:
      case CELL_VOLTAGE_10:
      case CELL_VOLTAGE_11:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            uint32_t index = (msg->id - CELL_VOLTAGE_0) * 8;
            // vm_msgs::msg::VoltageSet out;
            ams_report_msg_.cell_voltages[0 + index] = message->GetSignal("Cell" + ( 1 + index))->GetResult();
            ams_report_msg_.cell_voltages[1 + index] = message->GetSignal("Cell" + ( 2 + index))->GetResult();
            ams_report_msg_.cell_voltages[2 + index] = message->GetSignal("Cell" + ( 3 + index))->GetResult();
            ams_report_msg_.cell_voltages[3 + index] = message->GetSignal("Cell" + ( 4 + index))->GetResult();
            ams_report_msg_.cell_voltages[4 + index] = message->GetSignal("Cell" + ( 5 + index))->GetResult();
            ams_report_msg_.cell_voltages[5 + index] = message->GetSignal("Cell" + ( 6 + index))->GetResult();
            ams_report_msg_.cell_voltages[6 + index] = message->GetSignal("Cell" + ( 7 + index))->GetResult();
            ams_report_msg_.cell_voltages[7 + index] = message->GetSignal("Cell" + ( 8 + index))->GetResult();
            if(msg->id == CELL_VOLTAGE_11) {
              ams_report_msg_.header.stamp = msg->header.stamp;
              pub_ams_report_->publish(ams_report_msg_);
            }
          }

        }
        break;
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
      // pub_ams_report_->publish(ams_report_msg_);
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