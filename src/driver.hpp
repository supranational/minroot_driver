// Copyright Supranational LLC
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DRIVER_HPP__
#define __DRIVER_HPP__

#include <gmp.h>
#include "ftdi_driver.hpp"
#include "constants.h"

// Define the baseclass VDF driver
class Driver {
public:
  Driver(double freq, double board_voltage);

  // Functions for serializing to buffers
  // Append size bytes to 'buf' starting at offset. Returns size.
  size_t write_bytes(size_t size, size_t offset, uint8_t *buf, uint32_t val);
  size_t write_bytes(size_t size, size_t offset, uint8_t *buf, uint64_t val);
  size_t write_bytes(size_t size, size_t offset, uint8_t *buf,
                     mpz_t val, size_t num_coeffs);

  // Functions for deserializing from buffers
  size_t read_bytes(size_t size, size_t offset, uint8_t *buf, uint32_t &val);
  size_t read_bytes(size_t size, size_t offset, uint8_t *buf, uint64_t &val);
  size_t read_bytes(size_t size, size_t offset, uint8_t *buf,
                    mpz_t val, size_t num_coeffs);

  // Returns the buffer size needed for a command
  size_t CmdSize() {
    return (VDF_CMD_START_REG_OFFSET - VDF_CMD_JOB_ID_REG_OFFSET + 4);
  }

  // Returns the buffer size needed for a status
  size_t StatusSize() {
    return (VDF_STATUS_END_REG_OFFSET - VDF_STATUS_ITER_0_REG_OFFSET);
  }

  // Perform a single dword register write, return 0 on success
  int RegWrite(uint32_t addr, uint32_t data) {
    uint32_t buf;
    int stat;
    write_bytes(REG_BYTES, 0, (uint8_t *)&buf, data);
    stat = ftdi.WriteCmd(addr, (uint8_t *)&buf, REG_BYTES);
    if (stat) {
      printf("ftdi.WriteCmd in RegWrite failed (error %d)\n",stat);
      return stat;
    };
    return 0;
  }

  // Perform a single dword register read, return 0 on success
  int RegRead(uint32_t addr, uint32_t &data) {
    uint32_t buf;
    int stat;
    stat = ftdi.ReadCmd(addr, (uint8_t *)&buf, REG_BYTES);
    if (stat) {
      printf("ftdi.ReadCmd in RegRead failed (error %d)\n",stat);
      return stat;
    };
    read_bytes(REG_BYTES, 0, (uint8_t *)&buf, data);
    return 0;
  }

  // Enable the engine at the provided address
  void EnableEngine(unsigned control_csr) {
    RegWrite(control_csr, (uint32_t)0);  // Clear reset
    RegWrite(control_csr, CLOCK_BIT);    // Enable clock
  }

  // Disable the engine at the provided address
  void DisableEngine(unsigned control_csr) {
    RegWrite(control_csr, (uint32_t)0);  // Disable clock
    RegWrite(control_csr, RESET_BIT);    // Set reset
  }

  // Reset the engine at the provided address
  void ResetEngine(unsigned control_csr) {
    DisableEngine(control_csr);
    EnableEngine(control_csr);
  }

  void InitializeDevice(double freq, double board_voltage);

  void SerializeJob(uint8_t *buf,
                    uint32_t job_id,
                    uint64_t iteration_count,
                    uint64_t starting_iteration,
                    mpz_t x,
                    mpz_t y);

  void DeserializeJob(uint8_t *buf,
                      uint32_t &job_id,
                      uint64_t &iteration_count,
                      mpz_t x,
                      mpz_t y);

  void EnablePvt();
  double ValueToTemp(uint32_t temp_code);
  uint32_t TempToValue(double temp);
  double GetPvtTemp();
  double ValueToVoltage(uint32_t temp_code);
  double GetPvtVoltage();
  void InterpretPvtBurst(uint8_t *d, double *temperature, double *voltage);
  double GetTempAlarmExternal();
  double GetTempAlarmEngine();
  bool IsTempAlarmExternalSet();
  bool IsTempAlarmEngineSet();
  bool IsTempAlarmSet();
  bool SetTempAlarmExternal(double temp);
  bool SetTempAlarmEngine(double temp);
  void ResetPLL();
  bool SetPLLFrequency(double frequency);
  double GetPLLFrequency();
  int Reset(uint32_t sleep_duration);
  int TurnFanOn();
  int TurnFanOff();
  int I2CWriteReg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
  int I2CReadReg(uint8_t i2c_addr, uint8_t reg_addr, size_t len, uint8_t* data);
  double GetBoardVoltage();
  int SetBoardVoltage(double voltage);
  double GetBoardCurrent();
  double GetPower();

  FtdiDriver ftdi;
};
#endif
