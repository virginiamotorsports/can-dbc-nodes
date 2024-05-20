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
                
                inverter_report_msg_.flux_header.stamp = msg->header.stamp;
                inverter_report_msg_.inv_iq_command = message->GetSignal("INV_Iq_Command")->GetResult();
                inverter_report_msg_.inv_id_command = message->GetSignal("INV_Id_Command")->GetResult();
            //     inverter_report_msg_.INV_Flux_Weakening_Output = message->GetSignal("INV_Flux_Weakening_Output")->GetResult();
            //     inverter_report_msg_.INV_Modulation_Index = message->GetSignal("INV_Modulation_Index")->GetResult();
            }
        }
        break;

      case TORQUE_AND_TIMER_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.torque_header.stamp = msg->header.stamp;
                // inverter_report_msg_.INV_Power_On_Timer = message->GetSignal("INV_Power_On_Timer")->GetResult();
                inverter_report_msg_.inv_torque_feedback = message->GetSignal("INV_Torque_Feedback")->GetResult();
                inverter_report_msg_.inv_commanded_torque = message->GetSignal("INV_Commanded_Torque")->GetResult();
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

                inverter_report_msg_.command_header.stamp = msg->header.stamp;
                inverter_report_msg_.vcu_inv_inverter_enable = message->GetSignal("VCU_INV_Inverter_Enable")->GetResult();
                inverter_report_msg_.vcu_inv_direction_command = message->GetSignal("VCU_INV_Direction_Command")->GetResult();
                inverter_report_msg_.vcu_inv_speed_command = message->GetSignal("VCU_INV_Speed_Command")->GetResult();
                inverter_report_msg_.vcu_inv_torque_command = message->GetSignal("VCU_INV_Torque_Command")->GetResult();
                inverter_report_msg_.vcu_inv_inverter_discharge = message->GetSignal("VCU_INV_Inverter_Discharge")->GetResult();
                inverter_report_msg_.vcu_inv_torque_limit_command = message->GetSignal("VCU_INV_Torque_Limit_Command")->GetResult();
                inverter_report_msg_.vcu_inv_speed_mode_enable = message->GetSignal("VCU_INV_Speed_Mode_Enable")->GetResult();
                inverter_report_msg_.vcu_inv_rolling_counter = message->GetSignal("VCU_INV_Rolling_Counter")->GetResult();
            }
        }
        break;

      case FAULT_CODES:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.fault_header.stamp = msg->header.stamp;
                inverter_report_msg_.inv_run_fault_hi = message->GetSignal("INV_Run_Fault_Hi")->GetResult();
                inverter_report_msg_.inv_post_fault_hi = message->GetSignal("INV_Post_Fault_Hi")->GetResult();
                inverter_report_msg_.inv_run_fault_lo = message->GetSignal("INV_Run_Fault_Lo")->GetResult();
                inverter_report_msg_.inv_post_fault_lo = message->GetSignal("INV_Post_Fault_Lo")->GetResult();
            }
        }
        break;

      case INTERAL_STATES:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.internal_states_header.stamp = msg->header.stamp;
                inverter_report_msg_.inv_direction_command = message->GetSignal("INV_Direction_Command")->GetResult();
                inverter_report_msg_.inv_inverter_enable_state = message->GetSignal("INV_Inverter_Enable_State")->GetResult();
                // inverter_report_msg_.INV_Relay_3_Status = message->GetSignal("INV_Relay_3_Status")->GetResult();
                // inverter_report_msg_.INV_Relay_4_Status = message->GetSignal("INV_Relay_4_Status")->GetResult();
                // inverter_report_msg_.INV_Relay_2_Status = message->GetSignal("INV_Relay_2_Status")->GetResult();
                inverter_report_msg_.inv_inverter_run_mode = message->GetSignal("INV_Inverter_Run_Mode")->GetResult();
                inverter_report_msg_.inv_inverter_command_mode = message->GetSignal("INV_Inverter_Command_Mode")->GetResult();
                // inverter_report_msg_.INV_Relay_1_Status = message->GetSignal("INV_Relay_1_Status")->GetResult();
                inverter_report_msg_.inv_inverter_state = message->GetSignal("INV_Inverter_State")->GetResult();
                inverter_report_msg_.inv_vsm_state = message->GetSignal("INV_VSM_State")->GetResult();
                inverter_report_msg_.inv_inverter_enable_lockout = message->GetSignal("INV_Inverter_Enable_Lockout")->GetResult();
                inverter_report_msg_.inv_inverter_discharge_state = message->GetSignal("INV_Inverter_Discharge_State")->GetResult();
                // inverter_report_msg_.INV_Relay_5_Status = message->GetSignal("INV_Relay_5_Status")->GetResult();
                // inverter_report_msg_.INV_Relay_6_Status = message->GetSignal("INV_Relay_6_Status")->GetResult();
                inverter_report_msg_.inv_bms_active = message->GetSignal("INV_BMS_Active")->GetResult();
                inverter_report_msg_.inv_bms_torque_limiting = message->GetSignal("INV_BMS_Torque_Limiting")->GetResult();
                inverter_report_msg_.inv_pwm_frequency = message->GetSignal("INV_PWM_Frequency")->GetResult();
                inverter_report_msg_.inv_limit_max_speed = message->GetSignal("INV_Limit_Max_Speed")->GetResult();
                inverter_report_msg_.inv_limit_hot_spot = message->GetSignal("INV_Limit_Hot_Spot")->GetResult();
                inverter_report_msg_.inv_low_speed_limiting = message->GetSignal("INV_Low_Speed_Limiting")->GetResult();
                inverter_report_msg_.inv_rolling_counter = message->GetSignal("INV_Rolling_Counter")->GetResult();
                inverter_report_msg_.inv_limit_coolant_derating = message->GetSignal("INV_Limit_Coolant_Derating")->GetResult();
                inverter_report_msg_.inv_self_sensing_assist_enable = message->GetSignal("INV_Self_Sensing_Assist_Enable")->GetResult();
                inverter_report_msg_.inv_limit_stall_burst_model = message->GetSignal("INV_Limit_Stall_Burst_Model")->GetResult();
                inverter_report_msg_.inv_burst_model_mode = message->GetSignal("INV_Burst_Model_Mode")->GetResult();
                inverter_report_msg_.inv_key_switch_start_status = message->GetSignal("INV_Key_Switch_Start_Status")->GetResult();
            }
        }
        break;

      case INTERNAL_VOLTAGES:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_ref_voltage_12_0 = message->GetSignal("INV_Ref_Voltage_12_0")->GetResult();
                inverter_report_msg_.inv_ref_voltage_5_0 = message->GetSignal("INV_Ref_Voltage_5_0")->GetResult();
                inverter_report_msg_.inv_ref_voltage_2_5 = message->GetSignal("INV_Ref_Voltage_2_5")->GetResult();
                inverter_report_msg_.inv_ref_voltage_1_5 = message->GetSignal("INV_Ref_Voltage_1_5")->GetResult();
            }
        }
        break;

      case FLUX_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_iq = message->GetSignal("INV_Iq")->GetResult();
                inverter_report_msg_.inv_id = message->GetSignal("INV_Id")->GetResult();
                inverter_report_msg_.inv_vq_ff = message->GetSignal("INV_Vq_ff")->GetResult();
                inverter_report_msg_.inv_vd_ff = message->GetSignal("INV_Vd_ff")->GetResult();
            }
        }
        break;

      case VOLTAGE_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_vbc_vq_voltage = message->GetSignal("INV_VBC_Vq_Voltage")->GetResult();
                inverter_report_msg_.inv_vab_vd_voltage = message->GetSignal("INV_VAB_Vd_Voltage")->GetResult();
                inverter_report_msg_.inv_output_voltage = message->GetSignal("INV_Output_Voltage")->GetResult();
                inverter_report_msg_.inv_dc_bus_voltage = message->GetSignal("INV_DC_Bus_Voltage")->GetResult();
            }
        }
        break;

      case CURRENT_INFO:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_dc_bus_current = message->GetSignal("INV_DC_Bus_Current")->GetResult();
                inverter_report_msg_.inv_phase_c_current = message->GetSignal("INV_Phase_C_Current")->GetResult();
                inverter_report_msg_.inv_phase_b_current = message->GetSignal("INV_Phase_B_Current")->GetResult();
                inverter_report_msg_.inv_phase_a_current = message->GetSignal("INV_Phase_A_Current")->GetResult();
            }
        }
        break;

      case MOTOR_POSITION:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_delta_resolver_filtered = message->GetSignal("INV_Delta_Resolver_Filtered")->GetResult();
                inverter_report_msg_.inv_electrical_output_frequency = message->GetSignal("INV_Electrical_Output_Frequency")->GetResult();
                inverter_report_msg_.inv_motor_speed = message->GetSignal("INV_Motor_Speed")->GetResult();
                inverter_report_msg_.inv_motor_angle_electrical = message->GetSignal("INV_Motor_Angle_Electrical")->GetResult();
            }
        }
        break;

      case DIGITAL_INPUT_STATUS:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                // inverter_report_msg_.INV_Digital_Input_5 = message->GetSignal("INV_Digital_Input_5")->GetResult();
                // inverter_report_msg_.INV_Digital_Input_4 = message->GetSignal("INV_Digital_Input_4")->GetResult();
                // inverter_report_msg_.INV_Digital_Input_3 = message->GetSignal("INV_Digital_Input_3")->GetResult();
                // inverter_report_msg_.INV_Digital_Input_2 = message->GetSignal("INV_Digital_Input_2")->GetResult();
                // inverter_report_msg_.INV_Digital_Input_1 = message->GetSignal("INV_Digital_Input_1")->GetResult();
                // inverter_report_msg_.INV_Digital_Input_6 = message->GetSignal("INV_Digital_Input_6")->GetResult();
                // inverter_report_msg_.INV_Digital_Input_7 = message->GetSignal("INV_Digital_Input_7")->GetResult();
                // inverter_report_msg_.INV_Digital_Input_8 = message->GetSignal("INV_Digital_Input_8")->GetResult();
            }
        }
        break;

      case ANALOG_INPUT_VOLTAGE:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                // inverter_report_msg_.INV_Analog_Input_1 = message->GetSignal("INV_Analog_Input_1")->GetResult();
                // inverter_report_msg_.INV_Analog_Input_2 = message->GetSignal("INV_Analog_Input_2")->GetResult();
                // inverter_report_msg_.INV_Analog_Input_3 = message->GetSignal("INV_Analog_Input_3")->GetResult();
                // inverter_report_msg_.INV_Analog_Input_4 = message->GetSignal("INV_Analog_Input_4")->GetResult();
                // inverter_report_msg_.INV_Analog_Input_5 = message->GetSignal("INV_Analog_Input_5")->GetResult();
                // inverter_report_msg_.INV_Analog_Input_6 = message->GetSignal("INV_Analog_Input_6")->GetResult();
            }
        }
        break;

      case TEMP_SET_3:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_torque_shudder = message->GetSignal("INV_Torque_Shudder")->GetResult();
                inverter_report_msg_.inv_motor_temp = message->GetSignal("INV_Motor_Temp")->GetResult();
                inverter_report_msg_.inv_hot_spot_temp = message->GetSignal("INV_Hot_Spot_Temp")->GetResult();
                inverter_report_msg_.inv_coolant_temp = message->GetSignal("INV_Coolant_Temp")->GetResult();
            }
        }
        break;

      case TEMP_SET_2:
        {
          NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_rtd2_temperature = message->GetSignal("INV_RTD2_Temperature")->GetResult();
                inverter_report_msg_.inv_rtd1_temperature = message->GetSignal("INV_RTD1_Temperature")->GetResult();
                inverter_report_msg_.inv_control_board_temp = message->GetSignal("INV_Control_Board_Temp")->GetResult();
                inverter_report_msg_.inv_stall_burst_model_temp = message->GetSignal("INV_Stall_Burst_Model_Temp")->GetResult();
            }
        }
        break;

      case TEMP_SET_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_gate_driver_board_temp = message->GetSignal("INV_Gate_Driver_Board_Temp")->GetResult();
                inverter_report_msg_.inv_module_c_temp = message->GetSignal("INV_Module_C_Temp")->GetResult();
                inverter_report_msg_.inv_module_b_temp = message->GetSignal("INV_Module_B_Temp")->GetResult();
                inverter_report_msg_.inv_module_a_temp = message->GetSignal("INV_Module_A_Temp")->GetResult();
            }
        }
        break;

      case FIRMWARE_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_project_code_eep_ver = message->GetSignal("INV_Project_Code_EEP_Ver")->GetResult();
                inverter_report_msg_.inv_sw_version = message->GetSignal("INV_SW_Version")->GetResult();
                inverter_report_msg_.inv_datecode_mmdd = message->GetSignal("INV_DateCode_MMDD")->GetResult();
                inverter_report_msg_.inv_datecode_yyyy = message->GetSignal("INV_DateCode_YYYY")->GetResult();
            }
        }
        break;

      case DIAGNOSTIC_DATA_MSGS:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.diag_header.stamp = msg->header.stamp;
                inverter_report_msg_.inv_diag_record = message->GetSignal("INV_Diag_Record")->GetResult();
                inverter_report_msg_.inv_diag_segment = message->GetSignal("INV_Diag_Segment")->GetResult();
                inverter_report_msg_.inv_diag_gamma_resolver = message->GetSignal("INV_Diag_Gamma_Resolver")->GetResult();
                inverter_report_msg_.inv_diag_gamma_observer = message->GetSignal("INV_Diag_Gamma_Observer")->GetResult();
                inverter_report_msg_.inv_diag_sin_used = message->GetSignal("INV_Diag_Sin_Used")->GetResult();
                inverter_report_msg_.inv_diag_cos_used = message->GetSignal("INV_Diag_Cos_Used")->GetResult();
                inverter_report_msg_.inv_diag_ia = message->GetSignal("INV_Diag_Ia")->GetResult();
                inverter_report_msg_.inv_diag_ib = message->GetSignal("INV_Diag_Ib")->GetResult();
                inverter_report_msg_.inv_diag_ic = message->GetSignal("INV_Diag_Ic")->GetResult();
                inverter_report_msg_.inv_diag_vdc = message->GetSignal("INV_Diag_Vdc")->GetResult();
                inverter_report_msg_.inv_diag_iq_cmd = message->GetSignal("INV_Diag_Iq_cmd")->GetResult();
                inverter_report_msg_.inv_diag_id_cmd = message->GetSignal("INV_Diag_Id_cmd")->GetResult();
                inverter_report_msg_.inv_diag_mod_index = message->GetSignal("INV_Diag_Mod_Index")->GetResult();
                inverter_report_msg_.inv_diag_fw_output = message->GetSignal("INV_Diag_FW_Output")->GetResult();
                inverter_report_msg_.inv_diag_vq_cmd = message->GetSignal("INV_Diag_Vq_Cmd")->GetResult();
                inverter_report_msg_.inv_diag_vd_cmd = message->GetSignal("INV_Diag_Vd_Cmd")->GetResult();
                inverter_report_msg_.inv_diag_vqs_cmd = message->GetSignal("INV_Diag_Vqs_Cmd")->GetResult();
                inverter_report_msg_.inv_diag_pwm_freq = message->GetSignal("INV_Diag_PWM_Freq")->GetResult();
                inverter_report_msg_.inv_diag_run_faults_lo = message->GetSignal("INV_Diag_Run_Faults_Lo")->GetResult();
                inverter_report_msg_.inv_diag_run_faults_hi = message->GetSignal("INV_Diag_Run_Faults_Hi")->GetResult();
            }
        }
        break;

      case FAST_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_fast_torque_command = message->GetSignal("INV_Fast_Torque_Command")->GetResult();
                inverter_report_msg_.inv_fast_torque_feedback = message->GetSignal("INV_Fast_Torque_Feedback")->GetResult();
                inverter_report_msg_.inv_fast_motor_speed = message->GetSignal("INV_Fast_Motor_Speed")->GetResult();
                inverter_report_msg_.inv_fast_dc_bus_voltage = message->GetSignal("INV_Fast_DC_Bus_Voltage")->GetResult();
            }
        }
        break;

      case TORQUE_CAPABILITY:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.inv_torque_capability = message->GetSignal("INV_Torque_Capability")->GetResult();
            }
        }
        break;
        
      case CURRENT_LIMIT:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                inverter_report_msg_.bms_max_discharge_current = message->GetSignal("BMS_Max_Discharge_Current")->GetResult();
                inverter_report_msg_.bms_max_charge_current = message->GetSignal("BMS_Max_Charge_Current")->GetResult();
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

      case 2196807680: // M100_VCU_States1
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                vcu_report_msg_.run_mode_state = message->GetSignal("Run_Mode_State")->GetResult();
                vcu_report_msg_.drive_mode_state = message->GetSignal("Drive_Mode_State")->GetResult();
                vcu_report_msg_.run_time = message->GetSignal("Run_Time")->GetResult();
                vcu_report_msg_.stop_command_state = message->GetSignal("Stop_Command_State")->GetResult();
                vcu_report_msg_.start_command_state = message->GetSignal("Start_Command_State")->GetResult();
                vcu_report_msg_.opstate = message->GetSignal("OPState")->GetResult();
                vcu_report_msg_.mc2dischargecmd = message->GetSignal("MC2DischargeCmd")->GetResult();
                vcu_report_msg_.mc1dischargecmd = message->GetSignal("MC1DischargeCmd")->GetResult();
                vcu_report_msg_.mcpower = message->GetSignal("MCPower")->GetResult();
                vcu_report_msg_.mc1enable = message->GetSignal("MC1Enable")->GetResult();
                vcu_report_msg_.start_safe = message->GetSignal("Start_Safe")->GetResult();
                vcu_report_msg_.mc1contenable = message->GetSignal("MC1ContEnable")->GetResult();
                vcu_report_msg_.pumpcont_xcheck = message->GetSignal("PumpCont_XCheck")->GetResult();
                vcu_report_msg_.hvil_charge = message->GetSignal("HVIL_Charge")->GetResult();
                vcu_report_msg_.hvil_main = message->GetSignal("HVIL_Main")->GetResult();
                vcu_report_msg_.mc1_posfb = message->GetSignal("MC1_PosFB")->GetResult();
                vcu_report_msg_.mc1_negfb = message->GetSignal("MC1_NegFB")->GetResult();
                vcu_report_msg_.mc1_hvdetect = message->GetSignal("MC1_HVDetect")->GetResult();
                vcu_report_msg_.mc1_dcvoltagesafestate = message->GetSignal("MC1_DCVoltageSafeState")->GetResult();
                vcu_report_msg_.mc1_prechgfb = message->GetSignal("MC1_PreChgFB")->GetResult();
                vcu_report_msg_.performance_level = message->GetSignal("Performance_Level")->GetResult();
            }
        }
        break;

        case 2196807682: // M102_VCU_States2
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                vcu_report_msg_.vcu_12v_input = message->GetSignal("VCU_12V_Input")->GetResult();
                vcu_report_msg_.mc1_prechg_cmd = message->GetSignal("MC1_PreChg_Cmd")->GetResult();
                vcu_report_msg_.mc1_pos_cmd = message->GetSignal("MC1_Pos_Cmd")->GetResult();
                vcu_report_msg_.vcu_5v_output = message->GetSignal("VCU_5V_Output")->GetResult();
                vcu_report_msg_.mc1_prechgcomplete = message->GetSignal("MC1_PreChgComplete")->GetResult();
                vcu_report_msg_.speedmode_allowed = message->GetSignal("SpeedMode_Allowed")->GetResult();
                vcu_report_msg_.invcontrol_mode = message->GetSignal("InvControl_Mode")->GetResult();
                vcu_report_msg_.launch_mode = message->GetSignal("Launch_Mode")->GetResult();
                vcu_report_msg_.burnout_mode = message->GetSignal("Burnout_Mode")->GetResult();
                vcu_report_msg_.idle_mode = message->GetSignal("Idle_Mode")->GetResult();
                vcu_report_msg_.creep_mode = message->GetSignal("Creep_Mode")->GetResult();
                vcu_report_msg_.linelock_cntrl = message->GetSignal("LineLock_Cntrl")->GetResult();
                vcu_report_msg_.transbrake_cntrl = message->GetSignal("TransBrake_Cntrl")->GetResult();
                vcu_report_msg_.cool_pump_wake = message->GetSignal("Cool_Pump_Wake")->GetResult();
                vcu_report_msg_.cool_pump2_cntrl = message->GetSignal("Cool_Pump2_Cntrl")->GetResult();
                vcu_report_msg_.cool_pump1_cntrl = message->GetSignal("Cool_Pump1_Cntrl")->GetResult();
                vcu_report_msg_.oilpump1_on = message->GetSignal("OilPump1_On")->GetResult();
                vcu_report_msg_.cool_fan2_on = message->GetSignal("Cool_Fan2_On")->GetResult();
                vcu_report_msg_.cool_fan1_on = message->GetSignal("Cool_Fan1_On")->GetResult();
                vcu_report_msg_.cool_pumpspdtarget = message->GetSignal("Cool_PumpSpdTarget")->GetResult();
                vcu_report_msg_.brake_lampscntrl = message->GetSignal("Brake_LampsCntrl")->GetResult();
                vcu_report_msg_.transbrake_switch = message->GetSignal("TransBrake_Switch")->GetResult();
                vcu_report_msg_.linelock_switch = message->GetSignal("LineLock_Switch")->GetResult();
                vcu_report_msg_.inertia_switch = message->GetSignal("Inertia_Switch")->GetResult();
                vcu_report_msg_.imdstate = message->GetSignal("IMDState")->GetResult();
                vcu_report_msg_.shiftsol_cntrl = message->GetSignal("ShiftSol_Cntrl")->GetResult();
                vcu_report_msg_.shift_sol2 = message->GetSignal("Shift_Sol2")->GetResult();
                vcu_report_msg_.shift_sol1 = message->GetSignal("Shift_Sol1")->GetResult();
                vcu_report_msg_.contactors_enabled = message->GetSignal("Contactors_Enabled")->GetResult();
                vcu_report_msg_.chargecontactorcntrl = message->GetSignal("ChargeContactorCntrl")->GetResult();
                vcu_report_msg_.acc_powercntrl = message->GetSignal("Acc_PowerCntrl")->GetResult();
                vcu_report_msg_.acc_lightcntrl = message->GetSignal("Acc_LightCntrl")->GetResult();
                vcu_report_msg_.reverselamps_cntrl = message->GetSignal("ReverseLamps_Cntrl")->GetResult();
                vcu_report_msg_.parklamps_cntrl = message->GetSignal("ParkLamps_Cntrl")->GetResult();
                vcu_report_msg_.hvsafetylight_cntrl = message->GetSignal("HVSafetyLight_Cntrl")->GetResult();
                vcu_report_msg_.head_lampscntrl = message->GetSignal("Head_LampsCntrl")->GetResult();
            }
        }
        break;

        case 2196807684: // M104_VCU_States3
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                vcu_report_msg_.limmult_mc1temp_active = message->GetSignal("LimMult_MC1Temp_Active")->GetResult();
                vcu_report_msg_.limmult_mc1curr_active = message->GetSignal("LimMult_MC1Curr_Active")->GetResult();
                vcu_report_msg_.limmult_launch_time_active = message->GetSignal("LimMult_Launch_Time_Active")->GetResult();
                vcu_report_msg_.limmult_driveshaft_spd_active = message->GetSignal("LimMult_DriveShaft_Spd_Active")->GetResult();
                vcu_report_msg_.limmult_cellvolt_min_active = message->GetSignal("LimMult_CellVolt_Min_Active")->GetResult();
                vcu_report_msg_.limmult_cellvolt_max_active = message->GetSignal("LimMult_CellVolt_Max_Active")->GetResult();
                vcu_report_msg_.limmult_batt_soc_active = message->GetSignal("LimMult_Batt_SOC_Active")->GetResult();
                vcu_report_msg_.limmult_batt_dcl_active = message->GetSignal("LimMult_Batt_DCL_Active")->GetResult();
                vcu_report_msg_.limmult_motor1_temp_active = message->GetSignal("LimMult_Motor1_Temp_Active")->GetResult();
                vcu_report_msg_.limmult_motor1_spdlo_active = message->GetSignal("LimMult_Motor1_SpdLo_Active")->GetResult();
                vcu_report_msg_.limmult_pack_current_active = message->GetSignal("LimMult_Pack_Current_Active")->GetResult();
                vcu_report_msg_.limmult_over_rev_active = message->GetSignal("LimMult_Over_rev_Active")->GetResult();
                vcu_report_msg_.limmult_shift5_tq_active = message->GetSignal("LimMult_Shift5_Tq_Active")->GetResult();
                vcu_report_msg_.limmult_shift4_tq_active = message->GetSignal("LimMult_Shift4_Tq_Active")->GetResult();
                vcu_report_msg_.limmult_shift3_tq_active = message->GetSignal("LimMult_Shift3_Tq_Active")->GetResult();
                vcu_report_msg_.limmult_shift2_tq_active = message->GetSignal("LimMult_Shift2_Tq_Active")->GetResult();
                vcu_report_msg_.limmult_shift1_tq_active = message->GetSignal("LimMult_Shift1_Tq_Active")->GetResult();
                vcu_report_msg_.limmult_pack_voltage_active = message->GetSignal("LimMult_Pack_Voltage_Active")->GetResult();
                vcu_report_msg_.limmult_pack_templo_active = message->GetSignal("LimMult_Pack_TempLo_Active")->GetResult();
                vcu_report_msg_.limmult_pack_temphi_active = message->GetSignal("LimMult_Pack_TempHi_Active")->GetResult();
                vcu_report_msg_.launchtimer_running = message->GetSignal("LaunchTimer_Running")->GetResult();
                vcu_report_msg_.limmult_mc1currramp_active = message->GetSignal("LimMult_MC1CurrRamp_Active")->GetResult();
                vcu_report_msg_.limmult_vehspd_lo_active = message->GetSignal("LimMult_VehSpd_Lo_Active")->GetResult();
                vcu_report_msg_.limmult_vehspd_hi_active = message->GetSignal("LimMult_VehSpd_Hi_Active")->GetResult();
                vcu_report_msg_.launchramp_time = message->GetSignal("LaunchRamp_Time")->GetResult();
                vcu_report_msg_.run_time_counter = message->GetSignal("Run_Time_Counter")->GetResult();
            }
        }
        break;
      
      case 2196807712: // M120_MotorTorqueData1
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                vcu_report_msg_.motor1_torque_request = message->GetSignal("Motor1_Torque_Request")->GetResult();
                vcu_report_msg_.motor1_tqlimhi = message->GetSignal("Motor1_TqLimHi")->GetResult();
                vcu_report_msg_.motor1_tqlimlo = message->GetSignal("Motor1_TqLimLo")->GetResult();
                vcu_report_msg_.motor1_tqtable = message->GetSignal("Motor1_TqTable")->GetResult();
                vcu_report_msg_.motor1_tqlimmulthi = message->GetSignal("Motor1_TqLimMultHi")->GetResult();
                vcu_report_msg_.motor1_tqlimmultlo = message->GetSignal("Motor1_TqLimMultLo")->GetResult();
            }
        }
        break;

        case 2196807714: // M122_MotorTorqueData2
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                vcu_report_msg_.motor1_creeptorque = message->GetSignal("Motor1_CreepTorque")->GetResult();
                vcu_report_msg_.motor1_reversetorque = message->GetSignal("Motor1_ReverseTorque")->GetResult();
                vcu_report_msg_.motor1_torquetrimtable = message->GetSignal("Motor1_TorqueTrimTable")->GetResult();
                vcu_report_msg_.motortqspd_feedforward = message->GetSignal("MotorTqSpd_FeedForward")->GetResult();
                vcu_report_msg_.pedaltqmult_tbl1 = message->GetSignal("PedalTqMult_Tbl1")->GetResult();
                vcu_report_msg_.pedaltqmult_tbl2 = message->GetSignal("PedalTqMult_Tbl2")->GetResult();
            }
        }
        break;

        case 2196807716: // M124_MotorTorqueData3
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                // vcu_report_msg_.motor2_torquerequest = message->GetSignal("Motor2_TorqueRequest")->GetResult();
                // vcu_report_msg_.motor2_tqlimhi = message->GetSignal("Motor2_TqLimHi")->GetResult();
                // vcu_report_msg_.motor2_tqlimlo = message->GetSignal("Motor2_TqLimLo")->GetResult();
                // vcu_report_msg_.motor2_tqtable = message->GetSignal("Motor2_TqTable")->GetResult();
                // vcu_report_msg_.motor2_tqlimmulthi = message->GetSignal("Motor2_TqLimMultHi")->GetResult();
                // vcu_report_msg_.motor2_tqlimmultlo = message->GetSignal("Motor2_TqLimMultLo")->GetResult();
            }
        }
        break;

        case 2196807718: // M126_MotorTorqueData4
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                // vcu_report_msg_.motor2_creeptorque = message->GetSignal("Motor2_CreepTorque")->GetResult();
                // vcu_report_msg_.motor2_reversetorque = message->GetSignal("Motor2_ReverseTorque")->GetResult();
                // vcu_report_msg_.motor2_torquetrimtable = message->GetSignal("Motor2_TorqueTrimTable")->GetResult();
                vcu_report_msg_.regenbrake_torque = message->GetSignal("RegenBrake_Torque")->GetResult();
                vcu_report_msg_.pedaltqmult_tbl3 = message->GetSignal("PedalTqMult_Tbl3")->GetResult();
                vcu_report_msg_.pedaltqmult_tbl4 = message->GetSignal("PedalTqMult_Tbl4")->GetResult();
            }
        }
        break;

        case 2196807720: // M128_MotorTorqueData5
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                // vcu_report_msg_.motor3_torquerequest = message->GetSignal("Motor3_TorqueRequest")->GetResult();
                // vcu_report_msg_.motor3_tqlimhi = message->GetSignal("Motor3_TqLimHi")->GetResult();
                // vcu_report_msg_.motor3_tqlimlo = message->GetSignal("Motor3_TqLimLo")->GetResult();
                // vcu_report_msg_.motor3_tqtable = message->GetSignal("Motor3_TqTable")->GetResult();
                // vcu_report_msg_.motor3_tqlimmulthi = message->GetSignal("Motor3_TqLimMultHi")->GetResult();
                // vcu_report_msg_.motor3_tqlimmultlo = message->GetSignal("Motor3_TqLimMultLo")->GetResult();
            }
        }
        break;

        case 2196807728: // M130_MotorTorqueData6
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                // vcu_report_msg_.motor3_creeptorque = message->GetSignal("Motor3_CreepTorque")->GetResult();
                // vcu_report_msg_.motor3_reversetorque = message->GetSignal("Motor3_ReverseTorque")->GetResult();
                // vcu_report_msg_.motor3_torquetrimtable = message->GetSignal("Motor3_TorqueTrimTable")->GetResult();
                // vcu_report_msg_.motor4_torquerequest = message->GetSignal("Motor4_TorqueRequest")->GetResult();
            }
        }
        break;

        case 2196807730: // M132_MotorTorqueData7
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                // vcu_report_msg_.motor4_tqlimhi = message->GetSignal("Motor4_TqLimHi")->GetResult();
                // vcu_report_msg_.motor4_tqlimlo = message->GetSignal("Motor4_TqLimLo")->GetResult();
                // vcu_report_msg_.motor4_tqtable = message->GetSignal("Motor4_TqTable")->GetResult();
                // vcu_report_msg_.motor4_creeptorque = message->GetSignal("Motor4_CreepTorque")->GetResult();
                // vcu_report_msg_.motor4_tqlimmulthi = message->GetSignal("Motor4_TqLimMultHi")->GetResult();
                // vcu_report_msg_.motor4_tqlimmultlo = message->GetSignal("Motor4_TqLimMultLo")->GetResult();
            }
        }
        break;

        case 2196807732: // M134_MotorTorqueData8
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                // vcu_report_msg_.motor4_reversetorque = message->GetSignal("Motor4_ReverseTorque")->GetResult();
                // vcu_report_msg_.motor4_torquetrimtable = message->GetSignal("Motor4_TorqueTrimTable")->GetResult();
                vcu_report_msg_.launch_torque_multiplier = message->GetSignal("Launch_Torque_Multiplier")->GetResult();
                vcu_report_msg_.launch_torque_time = message->GetSignal("Launch_Torque_Time")->GetResult();
            }
        }
        break;

        // Motor Speed Data Messages
        case 2196807734: // M136_MotorSpeedData1
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                vcu_report_msg_.idletarget_speed = message->GetSignal("IdleTarget_Speed")->GetResult();
                vcu_report_msg_.startramp_targetspeed = message->GetSignal("StartRamp_TargetSpeed")->GetResult();
                vcu_report_msg_.freerevtarget_speed = message->GetSignal("FreeRevTarget_Speed")->GetResult();
                vcu_report_msg_.burnouttargetspeed = message->GetSignal("BurnoutTargetSpeed")->GetResult();
            }
        }
        break;

        case 2196807736: // M138_MotorSpeedData2
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                vcu_report_msg_.launchtarget_speed = message->GetSignal("LaunchTarget_Speed")->GetResult();
                vcu_report_msg_.motor_targetspeed = message->GetSignal("Motor_TargetSpeed")->GetResult();
                vcu_report_msg_.speedcontrol_pid = message->GetSignal("SpeedControl_PID")->GetResult();
                vcu_report_msg_.speedcontrol_pid_error = message->GetSignal("SpeedControl_PID_Error")->GetResult();
            }
        }
        break;

        case 2196807744: // M140_MotorSpeedData3
        {
            NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(msg->id);
            if (msg->dlc >= message->GetDlc()) {
                message->SetFrame(msg);

                vcu_report_msg_.speedcontrol_pid_pterm = message->GetSignal("SpeedControl_PID_PTerm")->GetResult();
                vcu_report_msg_.speedcontrol_pid_iterm = message->GetSignal("SpeedControl_PID_ITerm")->GetResult();
                vcu_report_msg_.speedcontrol_pid_dterm = message->GetSignal("SpeedControl_PID_DTerm")->GetResult();
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