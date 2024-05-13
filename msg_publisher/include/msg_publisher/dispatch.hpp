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
  ID_WHEEL_SPEED_REPORT_DO      = 0x0514,
  ID_BRAKE_PRESSURE_REPORT_DO   = 0x0515,
  ID_ACCELERATOR_REPORT_DO      = 0x0516,
  ID_STEERING_REPORT_DO         = 0x0517,
  ID_MISC_REPORT_DO             = 0x0518,
  ID_STEERING_REPORT_EXTD       = 0x0520,
  ID_RC_TO_CT                   = 0x04B0,
  ID_CT2                        = 0x057D,
  ID_MARELLI                    = 0x04E2,
  ID_G_FORCE                    = 0x05AA,
  ID_BASE_TO_CAR_TIMING         = 0x04B8,
  ID_WHEEL_STRAIN_GAUGE         = 0x051E,
  ID_WHEEL_POTENTIOMETER        = 0x051F,
  ID_TIRE_PRESSURE_FL           = 0x0528,
  ID_TIRE_PRESSURE_FR           = 0x0529,
  ID_TIRE_PRESSURE_RL           = 0x052A,
  ID_TIRE_PRESSURE_RR           = 0x052B,
  ID_VEL_ACC_HIL                = 0x05DD,
  ID_POSITION_HEADING_HIL       = 0x05DC,
  ID_PT_REPORT_1                = 0x053C,
  ID_PT_REPORT_2                = 0x053D,
  ID_PT_REPORT_3                = 0x053F,
  ID_DIAGNOSTIC_REPORT          = 0x053E,
  ID_MYLAPS_REPORT_1            = 0x00E4,
  ID_MYLAPS_REPORT_2            = 0x00E2,
  // MOTEC REPORT
  MOTEC_REPORT_1               = 0x0100,
  MOTEC_REPORT_2               = 0x0101,
  MOTEC_REPORT_3               = 0x0102,
  MOTEC_REPORT_4               = 0x0103,
  ID_M1_GENERAL_5               = 0x0645,
  ID_M1_GENERAL_6               = 0x0648,
  ID_M1_GENERAL_7               = 0x0649,
  ID_M1_GENERAL_8               = 0x064C,
  ID_M1_GENERAL_9               = 0x064D,
  ID_M1_GENERAL_10              = 0x064F,
  ID_M1_GENERAL_11              = 0x0650,
  ID_M1_GENERAL_12              = 0x0651,
  ID_M1_GENERAL_13              = 0x0653,
  ID_M1_GENERAL_14              = 0x0659,
  ID_M1_GENERAL_15              = 0x0630,
  ID_M1_GENERAL_16              = 0x0631,
  ID_M1_GENERAL_17              = 0x0668,
  ID_M1_GENERAL_18              = 0x0018,
};

}  // namespace msg_publisher

#endif  // msg_publisher__DISPATCH_HPP_
