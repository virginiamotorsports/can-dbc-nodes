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

#ifndef msg_publisher__DISPATCH_HPP_
#define msg_publisher__DISPATCH_HPP_

#include <stdint.h>

namespace msg_publisher
{

#undef BUILD_ASSERT

enum
{
  //AMS MSGS
  STATUS = 0x100,
  FAULT,
  DASH_REPORT,
  CELL_TEMP_0,
  CELL_TEMP_1,
  CELL_TEMP_2,
  CELL_TEMP_3,
  CELL_TEMP_4,
  CELL_TEMP_5,
  CELL_VOLTAGE_0,
  CELL_VOLTAGE_1,
  CELL_VOLTAGE_2,
  CELL_VOLTAGE_3,
  CELL_VOLTAGE_4,
  CELL_VOLTAGE_5,
  CELL_VOLTAGE_6,
  CELL_VOLTAGE_7,
  CELL_VOLTAGE_8,
  CELL_VOLTAGE_9,
  CELL_VOLTAGE_10,
  CELL_VOLTAGE_11,
  // VCU MSGS

  // Inverter msgs
  TEMP_SET_1 = 0x0A0,
  TEMP_SET_2,
  TEMP_SET_3,
  ANALOG_INPUT_VOLTAGE,
  DIGITAL_INPUT_STATUS,
  MOTOR_POSITION,
  CURRENT_INFO,
  VOLTAGE_INFO,
  FLUX_INFO,
  INTERNAL_VOLTAGES,
  INTERAL_STATES,
  FAULT_CODES,
  TORQUE_AND_TIMER_INFO,
  MODULATION_AND_FLUX_INFO,
  FIRMWARE_INFO,
  DIAGNOSTIC_DATA_MSGS,
  FAST_INFO,
  TORQUE_CAPABILITY,
  COMMAND_MESSAGE = 0X0C0,
  READ_WRITE_PARAM_COMMAND,
  READ_WRITE_PARAM_RESPONSE,
  CURRENT_LIMIT = 0x202,

  
  // dash msgs
  

};

}  // namespace msg_publisher

#endif  // msg_publisher__DISPATCH_HPP_
