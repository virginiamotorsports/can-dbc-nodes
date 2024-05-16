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
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(173);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::ModulationAndFluxInfo out;
            // out.header.stamp = msg->header.stamp;
            // out.iq_command = message->GetSignal("INV_Iq_Command")->GetResult();
            // out.id_command = message->GetSignal("INV_Id_Command")->GetResult();
            // out.flux_weakening_output = message->GetSignal("INV_Flux_Weakening_Output")->GetResult();
            // out.modulation_index = message->GetSignal("INV_Modulation_Index")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case TORQUE_AND_TIMER_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(172);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::TorqueAndTimerInfo out;
            // out.header.stamp = msg->header.stamp;
            // out.power_on_timer = message->GetSignal("INV_Power_On_Timer")->GetResult();
            // out.torque_feedback = message->GetSignal("INV_Torque_Feedback")->GetResult();
            // out.commanded_torque = message->GetSignal("INV_Commanded_Torque")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case READ_WRITE_PARAM_RESPONSE:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(194);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::ReadWriteParamResponse out;
            // out.header.stamp = msg->header.stamp;
            // out.parameter_response_write_ok = message->GetSignal("INV_Parameter_Response_Write_OK")->GetResult();
            // out.parameter_response_data = message->GetSignal("INV_Parameter_Response_Data")->GetResult();
            // out.parameter_response_addr = message->GetSignal("INV_Parameter_Response_Addr")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case READ_WRITE_PARAM_COMMAND:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(193);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::ReadWriteParamCommand out;
            // out.header.stamp = msg->header.stamp;
            // out.parameter_data = message->GetSignal("VCU_INV_Parameter_Data")->GetResult();
            // out.parameter_rw_command = message->GetSignal("VCU_INV_Parameter_RW_Command")->GetResult();
            // out.parameter_address = message->GetSignal("VCU_INV_Parameter_Address")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case COMMAND_MESSAGE:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(192);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::CommandMessage out;
            // out.header.stamp = msg->header.stamp;
            // out.inverter_enable = message->GetSignal("VCU_INV_Inverter_Enable")->GetResult();
            // out.direction_command = message->GetSignal("VCU_INV_Direction_Command")->GetResult();
            // out.speed_command = message->GetSignal("VCU_INV_Speed_Command")->GetResult();
            // out.torque_command = message->GetSignal("VCU_INV_Torque_Command")->GetResult();
            // out.inverter_discharge = message->GetSignal("VCU_INV_Inverter_Discharge")->GetResult();
            // out.torque_limit_command = message->GetSignal("VCU_INV_Torque_Limit_Command")->GetResult();
            // out.speed_mode_enable = message->GetSignal("VCU_INV_Speed_Mode_Enable")->GetResult();
            // out.rolling_counter = message->GetSignal("VCU_INV_Rolling_Counter")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case FAULT_CODES:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(171);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::FaultCodes out;
            // out.header.stamp = msg->header.stamp;
            // out.run_fault_hi = message->GetSignal("INV_Run_Fault_Hi")->GetResult();
            // out.post_fault_hi = message->GetSignal("INV_Post_Fault_Hi")->GetResult();
            // out.run_fault_lo = message->GetSignal("INV_Run_Fault_Lo")->GetResult();
            // out.post_fault_lo = message->GetSignal("INV_Post_Fault_Lo")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case INTERAL_STATES:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(170);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::InternalStates out;
            // out.header.stamp = msg->header.stamp;
            // out.direction_command = message->GetSignal("INV_Direction_Command")->GetResult();
            // out.inverter_enable_state = message->GetSignal("INV_Inverter_Enable_State")->GetResult();
            // out.relay_3_status = message->GetSignal("INV_Relay_3_Status")->GetResult();
            // out.relay_4_status = message->GetSignal("INV_Relay_4_Status")->GetResult();
            // out.relay_2_status = message->GetSignal("INV_Relay_2_Status")->GetResult();
            // out.inverter_run_mode = message->GetSignal("INV_Inverter_Run_Mode")->GetResult();
            // out.inverter_command_mode = message->GetSignal("INV_Inverter_Command_Mode")->GetResult();
            // out.relay_1_status = message->GetSignal("INV_Relay_1_Status")->GetResult();
            // out.inverter_state = message->GetSignal("INV_Inverter_State")->GetResult();
            // out.vsm_state = message->GetSignal("INV_VSM_State")->GetResult();
            // out.inverter_enable_lockout = message->GetSignal("INV_Inverter_Enable_Lockout")->GetResult();
            // out.inverter_discharge_state = message->GetSignal("INV_Inverter_Discharge_State")->GetResult();
            // out.relay_5_status = message->GetSignal("INV_Relay_5_Status")->GetResult();
            // out.relay_6_status = message->GetSignal("INV_Relay_6_Status")->GetResult();
            // out.bms_active = message->GetSignal("INV_BMS_Active")->GetResult();
            // out.bms_torque_limiting = message->GetSignal("INV_BMS_Torque_Limiting")->GetResult();
            // out.pwm_frequency = message->GetSignal("INV_PWM_Frequency")->GetResult();
            // out.limit_max_speed = message->GetSignal("INV_Limit_Max_Speed")->GetResult();
            // out.limit_hot_spot = message->GetSignal("INV_Limit_Hot_Spot")->GetResult();
            // out.low_speed_limiting = message->GetSignal("INV_Low_Speed_Limiting")->GetResult();
            // out.rolling_counter = message->GetSignal("INV_Rolling_Counter")->GetResult();
            // out.limit_coolant_derating = message->GetSignal("INV_Limit_Coolant_Derating")->GetResult();
            // out.self_sensing_assist_enable = message->GetSignal("INV_Self_Sensing_Assist_Enable")->GetResult();
            // out.limit_stall_burst_model = message->GetSignal("INV_Limit_Stall_Burst_Model")->GetResult();
            // out.burst_model_mode = message->GetSignal("INV_Burst_Model_Mode")->GetResult();
            // out.key_switch_start_status = message->GetSignal("INV_Key_Switch_Start_Status")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case INTERNAL_VOLTAGES:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(169);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::InternalVoltages out;
            // out.header.stamp = msg->header.stamp;
            // out.ref_voltage_12_0 = message->GetSignal("INV_Ref_Voltage_12_0")->GetResult();
            // out.ref_voltage_5_0 = message->GetSignal("INV_Ref_Voltage_5_0")->GetResult();
            // out.ref_voltage_2_5 = message->GetSignal("INV_Ref_Voltage_2_5")->GetResult();
            // out.ref_voltage_1_5 = message->GetSignal("INV_Ref_Voltage_1_5")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case FLUX_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(168);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::FluxID_IQ_Info out;
            // out.header.stamp = msg->header.stamp;
            // out.iq = message->GetSignal("INV_Iq")->GetResult();
            // out.id = message->GetSignal("INV_Id")->GetResult();
            // out.vq_ff = message->GetSignal("INV_Vq_ff")->GetResult();
            // out.vd_ff = message->GetSignal("INV_Vd_ff")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case VOLTAGE_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(167);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::VoltageInfo out;
            // out.header.stamp = msg->header.stamp;
            // out.vbc_vq_voltage = message->GetSignal("INV_VBC_Vq_Voltage")->GetResult();
            // out.vab_vd_voltage = message->GetSignal("INV_VAB_Vd_Voltage")->GetResult();
            // out.output_voltage = message->GetSignal("INV_Output_Voltage")->GetResult();
            // out.dc_bus_voltage = message->GetSignal("INV_DC_Bus_Voltage")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case CURRENT_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(166);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::CurrentInfo out;
            // out.header.stamp = msg->header.stamp;
            // out.dc_bus_current = message->GetSignal("INV_DC_Bus_Current")->GetResult();
            // out.phase_c_current = message->GetSignal("INV_Phase_C_Current")->GetResult();
            // out.phase_b_current = message->GetSignal("INV_Phase_B_Current")->GetResult();
            // out.phase_a_current = message->GetSignal("INV_Phase_A_Current")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case MOTOR_POSITION:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(165);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::MotorPositionInfo out;
            // out.header.stamp = msg->header.stamp;
            // out.delta_resolver_filtered = message->GetSignal("INV_Delta_Resolver_Filtered")->GetResult();
            // out.electrical_output_frequency = message->GetSignal("INV_Electrical_Output_Frequency")->GetResult();
            // out.motor_speed = message->GetSignal("INV_Motor_Speed")->GetResult();
            // out.motor_angle_electrical = message->GetSignal("INV_Motor_Angle_Electrical")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case DIGITAL_INPUT_STATUS:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(164);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::DigitalInputStatus out;
            // out.header.stamp = msg->header.stamp;
            // out.digital_input_5 = message->GetSignal("INV_Digital_Input_5")->GetResult();
            // out.digital_input_4 = message->GetSignal("INV_Digital_Input_4")->GetResult();
            // out.digital_input_3 = message->GetSignal("INV_Digital_Input_3")->GetResult();
            // out.digital_input_2 = message->GetSignal("INV_Digital_Input_2")->GetResult();
            // out.digital_input_1 = message->GetSignal("INV_Digital_Input_1")->GetResult();
            // out.digital_input_6 = message->GetSignal("INV_Digital_Input_6")->GetResult();
            // out.digital_input_7 = message->GetSignal("INV_Digital_Input_7")->GetResult();
            // out.digital_input_8 = message->GetSignal("INV_Digital_Input_8")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case ANALOG_INPUT_VOLTAGE:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(163);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::AnalogInputVoltages out;
            // out.header.stamp = msg->header.stamp;
            // out.analog_input_1 = message->GetSignal("INV_Analog_Input_1")->GetResult();
            // out.analog_input_2 = message->GetSignal("INV_Analog_Input_2")->GetResult();
            // out.analog_input_3 = message->GetSignal("INV_Analog_Input_3")->GetResult();
            // out.analog_input_4 = message->GetSignal("INV_Analog_Input_4")->GetResult();
            // out.analog_input_5 = message->GetSignal("INV_Analog_Input_5")->GetResult();
            // out.analog_input_6 = message->GetSignal("INV_Analog_Input_6")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case TEMP_SET_3:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(162);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::TemperatureSet3 out;
            // out.header.stamp = msg->header.stamp;
            // out.torque_shudder = message->GetSignal("INV_Torque_Shudder")->GetResult();
            // out.motor_temp = message->GetSignal("INV_Motor_Temp")->GetResult();
            // out.hot_spot_temp = message->GetSignal("INV_Hot_Spot_Temp")->GetResult();
            // out.coolant_temp = message->GetSignal("INV_Coolant_Temp")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case TEMP_SET_2:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(161);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::TemperatureSet2 out;
            // out.header.stamp = msg->header.stamp;
            // out.rtd2_temperature = message->GetSignal("INV_RTD2_Temperature")->GetResult();
            // out.rtd1_temperature = message->GetSignal("INV_RTD1_Temperature")->GetResult();
            // out.control_board_temp = message->GetSignal("INV_Control_Board_Temp")->GetResult();
            // out.stall_burst_model_temp = message->GetSignal("INV_Stall_Burst_Model_Temp")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case TEMP_SET_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(160);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::TemperatureSet1 out;
            // out.header.stamp = msg->header.stamp;
            // out.gate_driver_board_temp = message->GetSignal("INV_Gate_Driver_Board_Temp")->GetResult();
            // out.module_c_temp = message->GetSignal("INV_Module_C_Temp")->GetResult();
            // out.module_b_temp = message->GetSignal("INV_Module_B_Temp")->GetResult();
            // out.module_a_temp = message->GetSignal("INV_Module_A_Temp")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case FIRMWARE_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(174);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::FirmwareInfo out;
            // out.header.stamp = msg->header.stamp;
            // out.project_code_eep_ver = message->GetSignal("INV_Project_Code_EEP_Ver")->GetResult();
            // out.sw_version = message->GetSignal("INV_SW_Version")->GetResult();
            // out.datecode_mmdd = message->GetSignal("INV_DateCode_MMDD")->GetResult();
            // out.datecode_yyyy = message->GetSignal("INV_DateCode_YYYY")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case DIAGNOSTIC_DATA_MSGS:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(175);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::DiagDataMessage out;
            // out.header.stamp = msg->header.stamp;
            // out.diag_record = message->GetSignal("INV_Diag_Record")->GetResult();
            // out.diag_segment = message->GetSignal("INV_Diag_Segment")->GetResult();
            // out.diag_gamma_resolver = message->GetSignal("INV_Diag_Gamma_Resolver")->GetResult();
            // out.diag_gamma_observer = message->GetSignal("INV_Diag_Gamma_Observer")->GetResult();
            // out.diag_sin_used = message->GetSignal("INV_Diag_Sin_Used")->GetResult();
            // out.diag_cos_used = message->GetSignal("INV_Diag_Cos_Used")->GetResult();
            // out.diag_ia = message->GetSignal("INV_Diag_Ia")->GetResult();
            // out.diag_ib = message->GetSignal("INV_Diag_Ib")->GetResult();
            // out.diag_ic = message->GetSignal("INV_Diag_Ic")->GetResult();
            // out.diag_vdc = message->GetSignal("INV_Diag_Vdc")->GetResult();
            // out.diag_iq_cmd = message->GetSignal("INV_Diag_Iq_cmd")->GetResult();
            // out.diag_id_cmd = message->GetSignal("INV_Diag_Id_cmd")->GetResult();
            // out.diag_mod_index = message->GetSignal("INV_Diag_Mod_Index")->GetResult();
            // out.diag_fw_output = message->GetSignal("INV_Diag_FW_Output")->GetResult();
            // out.diag_vq_cmd = message->GetSignal("INV_Diag_Vq_Cmd")->GetResult();
            // out.diag_vd_cmd = message->GetSignal("INV_Diag_Vd_Cmd")->GetResult();
            // out.diag_vqs_cmd = message->GetSignal("INV_Diag_Vqs_Cmd")->GetResult();
            // out.diag_pwm_freq = message->GetSignal("INV_Diag_PWM_Freq")->GetResult();
            // out.diag_run_faults_lo = message->GetSignal("INV_Diag_Run_Faults_Lo")->GetResult();
            // out.diag_run_faults_hi = message->GetSignal("INV_Diag_Run_Faults_Hi")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case FAST_INFO:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(176);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::FastInfo out;
            // out.header.stamp = msg->header.stamp;
            // out.fast_torque_command = message->GetSignal("INV_Fast_Torque_Command")->GetResult();
            // out.fast_torque_feedback = message->GetSignal("INV_Fast_Torque_Feedback")->GetResult();
            // out.fast_motor_speed = message->GetSignal("INV_Fast_Motor_Speed")->GetResult();
            // out.fast_dc_bus_voltage = message->GetSignal("INV_Fast_DC_Bus_Voltage")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;

      case TORQUE_CAPABILITY:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(177);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::TorqueCapability out;
            // out.header.stamp = msg->header.stamp;
            // out.torque_capability = message->GetSignal("INV_Torque_Capability")->GetResult();
            // pub_inverter_report_->publish(out);
          }
        }
        break;
        
      case CURRENT_LIMIT:
        {
         NewEagle::DbcMessage* message = dbwDbc_can0_.GetMessageById(514);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            // vm_msgs::msg::CurrentLimit out;
            // out.header.stamp = msg->header.stamp;
            // out.max_discharge_current = message->GetSignal("BMS_Max_Discharge_Current")->GetResult();
            // out.max_charge_current = message->GetSignal("BMS_Max_Charge_Current")->GetResult();
            // pub_inverter_report_->publish(out);
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