// Copyright Supranational LLC
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#define VDF_MULTIREG_COUNT                 10
#define VDF_CONTROL_REG_OFFSET             0x200000
#define VDF_CMD_JOB_ID_REG_OFFSET          0x201000
#define VDF_CMD_START_REG_OFFSET           0x201064
#define VDF_STATUS_JOB_ID_REG_OFFSET       0x202000
#define VDF_STATUS_ITER_0_REG_OFFSET       0x202004
#define VDF_STATUS_END_REG_OFFSET          0x20205c

#define PVT_ENABLE_REG_OFFSET              0x10000
#define PVT_PERIOD_REG_OFFSET              0x10004
#define PVT_TEMPERATURE_REG_OFFSET         0x10010
#define PVT_VOLTAGE_REG_OFFSET             0x10020
#define PVT_TEMP_ALARM_EXTERNAL_REG_OFFSET 0x10024
#define PVT_TEMP_ALARM_ENGINE_REG_OFFSET   0x10028

#define PVT_TEMP_ALARM_THRESHOLD_MASK      0x3ff
#define PVT_TEMP_ALARM_STATUS_BIT          15

#define PLL_CONTROL_REG_OFFSET             0x00
#define PLL_PRE_DIVIDE_REG_OFFSET          0x04
#define PLL_FB_DIVIDE_INTEGER_REG_OFFSET   0x08
#define PLL_POST_DIVIDE_REG_OFFSET         0x10
#define PLL_FILTER_RANGE_REG_OFFSET        0x14
#define PLL_STATUS_REG_OFFSET              0x1c

#define PLL_STATUS_LOCK_BIT                0
#define PLL_STATUS_DIVACK_BIT              1
#define PLL_CONTROL_RESET_BIT              0
#define PLL_CONTROL_BYPASS_BIT             1
#define PLL_CONTROL_NEWDIV_BIT             2
#define PLL_CONTROL_USEREF_BIT             3

#define WORD_BITS                          16
#define REDUNDANT_BITS                     17
#define REG_BITS                           32
#define REG_BYTES                          4
#define RESET_BIT                          0x10
#define CLOCK_BIT                          0x01
#define BURST_ADDR                         0x300000
#define VDF_ENGINE_STRIDE                  0x10000
#define NUM_1X_COEFFS                      17
#define ITERATION_BITS                     48

#endif  // __CONSTANTS_H__
