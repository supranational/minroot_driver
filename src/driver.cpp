// Copyright Supranational LLC
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <vector>
#include <cstring>
#include <cassert>
#include <cmath>
#include <unistd.h>
#include <stdexcept>
#include "driver.hpp"
#include "constants.h"
#include "pll_freqs.hpp"

#define VR_I2C_ADDR 0x38
#define CS_I2C_ADDR 0x70
#define IR_I2C_ADDR 0x48

Driver::Driver(double freq, double board_voltage) {
  InitializeDevice(freq, board_voltage);
}

size_t Driver::write_bytes(size_t size, size_t offset, uint8_t *buf,
                              uint32_t val) {
  // Insert bytes from val in big endian order
  uint8_t *p = (uint8_t *)&val;
  for (unsigned i = 0; i < REG_BYTES; i++) {
    buf[i + offset] = p[REG_BYTES - i - 1];
  }

  return size;
}

size_t Driver::write_bytes(size_t size, size_t offset, uint8_t *buf,
                              uint64_t val) {
  write_bytes(REG_BYTES, offset, buf, (uint32_t)val);
  write_bytes(REG_BYTES, offset + REG_BYTES, buf, (uint32_t)(val >> REG_BITS));
  return size;
}

size_t Driver::write_bytes(size_t size, size_t offset, uint8_t *buf,
                              mpz_t _val, size_t num_coeffs) {
  mpz_t tmp, val;
  mpz_inits(tmp, val, NULL);

  // Copy val so we don't alter it
  mpz_set(val, _val);

  // Two's complement signed values
  if (mpz_sgn(val) < 0) {
    unsigned bits = num_coeffs * WORD_BITS + REDUNDANT_BITS;
    mpz_t shifted;
    mpz_init(shifted);
    mpz_set_ui(shifted, 1);
    mpz_mul_2exp(shifted, shifted, bits); // left shift
    mpz_neg(val, val);
    mpz_sub(val, shifted, val);
    mpz_clear(shifted);
  }
  mpz_set(tmp, val);

  // Extract coefficients
  uint32_t word_mask = (1UL << WORD_BITS) - 1;
  std::vector<uint32_t> coeffs;
  for (size_t i = 0; i < num_coeffs - 1; i++) {
    uint32_t coeff = mpz_get_ui(tmp) & word_mask;
    coeffs.push_back(coeff);
    mpz_tdiv_q_2exp(tmp, tmp, WORD_BITS);
  }
  // Last coeff does not get masked
  coeffs.push_back(mpz_get_ui(tmp));

  // Pack coefficients, with redundant bits
  mpz_set_ui(tmp, 0);
  for (int i = num_coeffs - 1; i >= 0; i--) {
    mpz_mul_2exp(tmp, tmp, REDUNDANT_BITS); // left shift
    mpz_add_ui(tmp, tmp, coeffs[i]);
  }

  // Clear the buffer space
  memset(buf + offset, 0, size);

  // For simplicity assume size is divisible by 32 bits
  assert(size % REG_BYTES == 0);

  // Write 32 bits at a time
  uint64_t mask = (1ULL << REG_BITS) - 1;
  for (size_t count = 0; count < size; count += REG_BYTES) {
    write_bytes(REG_BYTES, offset + count, buf,
                (uint32_t)(mpz_get_ui(tmp) & mask));

    mpz_tdiv_q_2exp(tmp, tmp, REG_BITS);
  }

  mpz_clears(tmp, val, NULL);
  return size;
}

size_t Driver::read_bytes(size_t size, size_t offset, uint8_t *buf,
                             uint32_t &val) {
  // Reads bytes from buf into val in big endian order
  uint8_t *p = (uint8_t *)&val;
  for (unsigned i = 0; i < REG_BYTES; i++) {
    p[REG_BYTES - i - 1] = buf[i + offset];
  }

  return size;
}

size_t Driver::read_bytes(size_t size, size_t offset, uint8_t *buf,
                             uint64_t &val) {
  uint32_t val32;
  read_bytes(REG_BYTES, offset, buf, val32);
  val = val32;

  read_bytes(REG_BYTES, offset + REG_BYTES, buf, val32);
  val |= ((uint64_t)val32) << REG_BITS;

  return size;
}

size_t Driver::read_bytes(size_t size, size_t offset, uint8_t *buf,
                             mpz_t val, size_t num_coeffs) {
  mpz_t tmp, tmp2;
  mpz_inits(tmp, tmp2, NULL);

  // Gather uint32s
  std::vector<uint32_t> words;
  for (size_t count = 0; count < size; count += REG_BYTES) {
    uint32_t word;
    read_bytes(REG_BYTES, offset + count, buf, word);
    words.push_back(word);
  }

  // Pack redundant coeffs, most significant first
  mpz_set_ui(tmp, 0);
  for (int i = words.size() - 1; i >= 0; i--) {
    mpz_mul_2exp(tmp, tmp, REG_BITS); // left shift
    mpz_add_ui(tmp, tmp, words[i]);
  }

  // Unpack and reduce
  mpz_set_ui(val, 0);
  uint32_t coeff_mask = (1UL << REDUNDANT_BITS) - 1;
  for (size_t i = 0; i < num_coeffs && mpz_sgn(tmp) != 0; i++) {
    uint32_t coeff = mpz_get_ui(tmp) & coeff_mask;
    mpz_set_ui(tmp2, coeff);
    mpz_mul_2exp(tmp2, tmp2, WORD_BITS * i); // left shift
    mpz_add(val, val, tmp2);

    mpz_tdiv_q_2exp(tmp, tmp, REDUNDANT_BITS);
  }

  mpz_clears(tmp, tmp2, NULL);
  return size;
}

void Driver::InitializeDevice(double freq, double board_voltage) {
  if (ftdi.Open()) {
      throw std::runtime_error("Failed to open device");
  }

  int ret_val;

  ret_val = AutoConfigVr();
  if (ret_val != 0) {
    fprintf(stderr, "Aborting since AutoConfigVr() failed\n");
    abort();
  }

  ret_val = Reset(100);
  if (ret_val != 0) {
    fprintf(stderr, "Aborting since Reset() failed\n");
    abort();
  }

  ret_val = TurnFanOn();
  if (ret_val != 0) {
    fprintf(stderr, "Aborting since TurnFanOn() failed\n");
    abort();
  }

  // Adjust board voltage.
  double brd_voltage = GetBoardVoltage();
  printf("Board voltage is %1.3f V\n", brd_voltage);

  if (board_voltage > 1.0) {
    printf("Requested Board Voltage too high, adjusting\n");
    board_voltage = 1.0;
  } else if (board_voltage < 0.70) {
    printf("Requested Board Voltage too low, adjusting\n");
    board_voltage = 0.80;
  }

  ret_val = SetBoardVoltage(board_voltage);
  if (ret_val != 0) {
    fprintf(stderr, "Aborting since set voltage failed\n");
    abort();
  }

  brd_voltage = GetBoardVoltage();
  printf("Board voltage is now %1.3f V\n", brd_voltage);

  // Adjust engine frequency.
  bool set_status;
  if (freq < 100.0) {
    printf("Requested Frequency too low, adjusting\n");
    freq = 200.0;
  } else if (freq > 1100.0) {
    printf("Requested Frequency too high, adjusting\n");
    freq = 1000.0;
  }

  set_status = SetPLLFrequency(freq);
  if (set_status == false) {
    fprintf(stderr, "Aborting since freq not set\n");
    abort();
  }

  // Check frequency
  double freq_read = GetPLLFrequency();
  printf("Frequency is %f MHz\n", freq_read);

  // Report power and current.
  double brd_current = GetBoardCurrent();
  printf("Board current is %2.3f A\n", brd_current);

  double brd_power = GetPower();
  printf("Board power is %2.3f W\n", brd_power);

  // Enable PVT sensor
  EnablePvt();

  // Set external temperature alarm in PVT sensor
  double external_alarm_temp = 100.0;
  set_status = SetTempAlarmExternal(external_alarm_temp);
  if (set_status == false) {
    fprintf(stderr, "Aborting since temp alarm not set\n");
    abort();
  }

  // Set engine temperature alarm in PVT sensor
  double engine_alarm_temp = 100.0;
  set_status = SetTempAlarmEngine(engine_alarm_temp);
  if (set_status == false) {
    fprintf(stderr, "Aborting since temp alarm not set\n");
    abort();
  }
}

void Driver::SerializeJob(uint8_t *buf,
                          uint32_t job_id,
                          uint64_t iteration_count,
                          uint64_t starting_iteration,
                          mpz_t x,
                          mpz_t y) {
  size_t offset = 0;
  const size_t reg_size = REG_BYTES;
  offset += write_bytes(reg_size, offset, buf, job_id);
  offset += write_bytes(2 * reg_size, offset, buf, iteration_count);
  offset += write_bytes(2 * reg_size, offset, buf, starting_iteration);
  offset += write_bytes(reg_size * VDF_MULTIREG_COUNT,
                        offset, buf, x, NUM_1X_COEFFS);
  offset += write_bytes(reg_size * VDF_MULTIREG_COUNT,
                        offset, buf, y, NUM_1X_COEFFS);
  // The last word is 0x1 to start the job
  offset += write_bytes(reg_size, offset, buf, (uint32_t)0x1);
}

void Driver::DeserializeJob(uint8_t *buf,
                            uint32_t &job_id,
                            uint64_t &iteration_count,
                            mpz_t x,
                            mpz_t y) {
  size_t offset = 0;
  const size_t reg_size = REG_BYTES;
  const uint64_t iteration_count_mask = (1ULL << ITERATION_BITS) - 1;
  offset += read_bytes(reg_size, offset, buf, job_id);
  offset += read_bytes(2 * reg_size, offset, buf, iteration_count);
  iteration_count &= iteration_count_mask;
  offset += read_bytes(reg_size * VDF_MULTIREG_COUNT,
                       offset, buf, x, NUM_1X_COEFFS);
  offset += read_bytes(reg_size * VDF_MULTIREG_COUNT,
                       offset, buf, y, NUM_1X_COEFFS);
}

// Enable PVT
void Driver::EnablePvt() {
  uint32_t pvt_period = 83;
  RegWrite(PVT_PERIOD_REG_OFFSET, pvt_period);
  RegWrite(PVT_ENABLE_REG_OFFSET, (uint32_t)1);
}

// Calculate temperature (C) from input code
double Driver::ValueToTemp(uint32_t temp_code) {
  double a4 = -0.000000000008929;
  double a3 = 0.000000065714;
  double a2 = -0.00018002;
  double a1 = 0.33061;
  double a0 = -60.9267;

  double x4 = pow((double) temp_code, 4.0);
  double x3 = pow((double) temp_code, 3.0);
  double x2 = pow((double) temp_code, 2.0);

  double temp = (a4 * x4) + (a3 * x3) + (a2 * x2) +
                (a1 * (double) temp_code) + a0;
  return temp;
}

// Calculate code from temperature input (C)
uint32_t Driver::TempToValue(double temp) {
  double a4 = 0.000000027093;
  double a3 = 0.00002108;
  double a2 = 0.0076534;
  double a1 = 3.7764;
  double a0 = 205.64;

  double x4 = pow(temp, 4.0);
  double x3 = pow(temp, 3.0);
  double x2 = pow(temp, 2.0);

  uint32_t temp_code = (uint32_t)((a4 * x4) + (a3 * x3) + (a2 * x2) +
                                  (a1 * temp) + a0 + 0.5);

  return temp_code;
}

// Get temperature from PVT
double Driver::GetPvtTemp() {
  int ret_val;
  uint32_t temp_data;
  ret_val = RegRead(PVT_TEMPERATURE_REG_OFFSET, temp_data);

  if (ret_val != 0) {
    fprintf(stderr, "GetPvtTemp bad reg read %d\n", ret_val);
    return 0.0;
  }

  double temp = ValueToTemp(temp_data);

  return temp;
}

// Voltage from input code
double Driver::ValueToVoltage(uint32_t value) {
  double a1 = 0.00054903;
  double a0 = 0.45727;
  double voltage = (a1 * (double) value) + a0;
  return voltage;
}

// Get voltage from PVT
double Driver::GetPvtVoltage() {
  int ret_val;
  uint32_t voltage_data;
  ret_val = RegRead(PVT_VOLTAGE_REG_OFFSET, voltage_data);

  if (ret_val != 0) {
    fprintf(stderr, "GetPvtVoltage bad reg read %d\n", ret_val);
    return 0.0;
  }

  return ValueToVoltage(voltage_data);
}

void Driver::InterpretPvtBurst(uint8_t *d,
                               double *temperature,
                               double *voltage) {
  uint32 temp_code    = (d[0*4 + 0] << 24) |
                        (d[0*4 + 1] << 16) |
                        (d[0*4 + 2] <<  8) |
                        (d[0*4 + 3] <<  0);
  uint32 voltage_code = (d[4*4 + 0] << 24) |
                        (d[4*4 + 1] << 16) |
                        (d[4*4 + 2] <<  8) |
                        (d[4*4 + 3] <<  0);

  if (temperature != NULL) *temperature = ValueToTemp(temp_code);
  if (voltage != NULL) *voltage = ValueToVoltage(voltage_code);
}

// Get the programmed temperature external alarm (C)
double Driver::GetTempAlarmExternal() {
  int ret_val;
  uint32_t temp_data;
  ret_val = RegRead(PVT_TEMP_ALARM_EXTERNAL_REG_OFFSET, temp_data);

  if (ret_val != 0) {
    fprintf(stderr, "GetTempAlarmExternal bad reg read %d\n", ret_val);
    return 0.0;
  }

  temp_data &= PVT_TEMP_ALARM_THRESHOLD_MASK;

  double temp = ValueToTemp(temp_data);
  return temp;
}

// Get the programmed temperature engine alarm (C)
double Driver::GetTempAlarmEngine() {
  int ret_val;
  uint32_t temp_data;
  ret_val = RegRead(PVT_TEMP_ALARM_ENGINE_REG_OFFSET, temp_data);

  if (ret_val != 0) {
    fprintf(stderr, "GetTempAlarmEngine bad reg read %d\n", ret_val);
    return 0.0;
  }

  temp_data &= PVT_TEMP_ALARM_THRESHOLD_MASK;

  double temp = ValueToTemp(temp_data);
  return temp;
}

// Check if the temperature external alarm is triggered
bool Driver::IsTempAlarmExternalSet() {
  int ret_val;
  uint32_t temp_data;
  ret_val = RegRead(PVT_TEMP_ALARM_EXTERNAL_REG_OFFSET, temp_data);

  if (ret_val != 0) {
    fprintf(stderr, "GetTempAlarmExternal bad reg read %d\n", ret_val);
    return true;
  }

  return (((temp_data >> PVT_TEMP_ALARM_STATUS_BIT) & 0x1) == 1);
}

// Check if the temperature engine alarm is triggered
bool Driver::IsTempAlarmEngineSet() {
  int ret_val;
  uint32_t temp_data;
  ret_val = RegRead(PVT_TEMP_ALARM_ENGINE_REG_OFFSET, temp_data);

  if (ret_val != 0) {
    fprintf(stderr, "GetTempAlarmEngine bad reg read %d\n", ret_val);
    return true;
  }

  return (((temp_data >> PVT_TEMP_ALARM_STATUS_BIT) & 0x1) == 1);
}

bool Driver::IsTempAlarmSet() {
  bool eng_alarm = IsTempAlarmEngineSet();
  bool ext_alarm = IsTempAlarmExternalSet();
  return (eng_alarm || ext_alarm);
}

// Set the PVT temperature external alarm threshold
bool Driver::SetTempAlarmExternal(double temp) {
  uint32_t temp_code = TempToValue(temp);
  if (temp_code > 0x3FF) {
    fprintf(stderr, "SetTempAlarmExternal bad code %X from temp %lf\n",
            temp_code, temp);
    return false;
  }
  RegWrite(PVT_TEMP_ALARM_EXTERNAL_REG_OFFSET, temp_code);
  return true;
}

// Set the PVT temperature engine alarm threshold
bool Driver::SetTempAlarmEngine(double temp) {
  uint32_t temp_code = TempToValue(temp);
  if (temp_code > 0x3FF) {
    fprintf(stderr, "SetTempAlarmEngine bad code %X from temp %lf\n",
            temp_code, temp);
    return false;
  }
  RegWrite(PVT_TEMP_ALARM_ENGINE_REG_OFFSET, temp_code);
  return true;
}

void Driver::ResetPLL() {
  RegWrite(PLL_CONTROL_REG_OFFSET, (uint32_t)0x1); // Reset
}

bool Driver::SetPLLFrequency(double frequency, bool ss) {
  if (frequency > pll_entries[VALID_PLL_FREQS - 1].freq) {
    fprintf(stderr, "SetPLLFrequency frequency too high %lf\n", frequency);
    return false;
  }

  int entry_index = 0;
  while (frequency > pll_entries[entry_index].freq) {
    entry_index++;
  }

  uint32_t divr = pll_entries[entry_index].settings.divr;
  uint32_t divfi = pll_entries[entry_index].settings.divfi;
  uint32_t divq = pll_entries[entry_index].settings.divq;

  double ref_freq = 100.0 / (divr + 1);
  if (ref_freq > pll_filter_ranges[VALID_PLL_FILTER_RANGES - 1]) {
    fprintf(stderr, "SetPLLFrequency ref_freq too high %lf\n", ref_freq);
    return false;
  }

  uint32_t filter_range = 0;
  while (ref_freq >= pll_filter_ranges[filter_range]) {
    filter_range++;
  }

  //printf("Frequency %lf should use entry %d - divr %d fi %d q %d range %d\n",
  //       frequency, entry_index, divr, divfi, divq, filter_range);

  // Reset ASIC before changing frequency
  Reset(10000);

  RegWrite(PLL_CONTROL_REG_OFFSET, (uint32_t)0x1); // Reset

  RegWrite(PLL_PRE_DIVIDE_REG_OFFSET, divr);
  RegWrite(PLL_FB_DIVIDE_INTEGER_REG_OFFSET, divfi);
  RegWrite(PLL_POST_DIVIDE_REG_OFFSET, divq);
  RegWrite(PLL_FILTER_RANGE_REG_OFFSET, filter_range);

  if (ss) {
    uint32_t spread_spectrum =
      (
       // Enable
       (0x1 << PLL_SPREAD_SPECTRUM_ENABLE_BIT) |
       // Down spread
       (0x1 << PLL_SPREAD_SPECTRUM_DOWN_SPREAD_BIT) |
       // 4% depth
       (0x7 << PLL_SPREAD_SPECTRUM_MODULATION_DEPTH_OFFSET) |
       // 14.7 kHz modulation
       (0x5 << PLL_SPREAD_SPECTRUM_MODULATION_FREQUENCY_OFFSET)
       );
    RegWrite(PLL_SPREAD_SPECTRUM_REG_OFFSET, spread_spectrum);
  }

  RegWrite(PLL_CONTROL_REG_OFFSET, (uint32_t)0x4); // New div

  // Wait for ack on new div
  int ret_val;
  uint32_t pll_status = 0;
  int read_attempts = 0;
  while (((pll_status >> PLL_STATUS_DIVACK_BIT) & 0x1) == 0) {
    ret_val = RegRead(PLL_STATUS_REG_OFFSET, pll_status);
    read_attempts++;
    usleep(1000);
    if ((read_attempts > 4) || (ret_val != 0)) {
      fprintf(stderr, "SetPLLFrequency pll div never ack'd\n");
      return false;
    }
  }

  // Clear control reg
  RegWrite(PLL_CONTROL_REG_OFFSET, (uint32_t)0x0);

  // Wait for lock
  pll_status = 0;
  read_attempts = 0;
  while (((pll_status >> PLL_STATUS_LOCK_BIT) & 0x1) == 0) {
    ret_val = RegRead(PLL_STATUS_REG_OFFSET, pll_status);
    read_attempts++;
    usleep(1000);
    if ((read_attempts > 4) || (ret_val != 0)) {
      fprintf(stderr, "SetPLLFrequency pll never locked\n");
      return false;
    }
  }

  return true;
}

double Driver::GetPLLFrequency() {
  int ret_val;
  uint32_t pll_status = 0;
  ret_val = RegRead(PLL_CONTROL_REG_OFFSET, pll_status);

  if (ret_val != 0) {
    fprintf(stderr, "GetPLLFrequency bad reg read %d\n", ret_val);
    return 0.0;
  }

  //printf("GetPLLFrequency status %x\n", pll_status);
  if (((pll_status >> PLL_CONTROL_BYPASS_BIT) & 0x1) == 1) {
    return 100.0;
  } else if (((pll_status >> PLL_CONTROL_USEREF_BIT) & 0x1) == 1) {
    return 100.0;
  } else if (((pll_status >> PLL_CONTROL_RESET_BIT) & 0x1) == 1) {
    return 0.0;
  }

  uint32_t divr;
  uint32_t divfi;
  uint32_t divq;

  ret_val |= RegRead(PLL_PRE_DIVIDE_REG_OFFSET, divr);
  ret_val |= RegRead(PLL_FB_DIVIDE_INTEGER_REG_OFFSET, divfi);
  ret_val |= RegRead(PLL_POST_DIVIDE_REG_OFFSET, divq);

  //printf("Frequency registers - divr %d fi %d q %d\n", divr, divfi, divq);

  if (ret_val != 0) {
    fprintf(stderr, "GetPLLFrequency bad reg read %d\n", ret_val);
    return 0.0;
  }

  double ref_freq = 100.0 / (divr + 1);
  double vco_freq = ref_freq * (divfi + 1) * 4;
  double freq = vco_freq / ((divq + 1) * 2);
  return freq;
}

int Driver::Reset(uint32_t sleep_duration = 1000) {
  int ret_val;
  ret_val = ftdi.SetGPIO(GPIO_PORT2, 0);
  if (ret_val != 0) {
    fprintf(stderr, "Reset failed to set gpio, %d\n", ret_val);
    return ret_val;
  }
  usleep(sleep_duration);
  ret_val  = ftdi.SetGPIO(GPIO_PORT2, 1); // For open-drain outputs
  ret_val |= ftdi.TriGPIO(GPIO_PORT2);
  if (ret_val != 0) {
    fprintf(stderr, "Reset failed to tri-state gpio, %d\n", ret_val);
  }
  usleep(100000);
  return ret_val;
}

int Driver::TurnFanOn() {
  int ret_val;
  ret_val = ftdi.SetGPIO(GPIO_PORT3, 1);
  if (ret_val != 0) {
    fprintf(stderr, "TurnFanOn failed to set gpio, %d\n", ret_val);
  } else {
    usleep(2000000); // Allow the fan to spin up
  }
  return ret_val;
}

int Driver::TurnFanOff() {
  int ret_val;
  ret_val = ftdi.SetGPIO(GPIO_PORT3, 0);
  if (ret_val != 0) {
    fprintf(stderr, "TurnFanOff failed to set gpio, %d\n", ret_val);
  }
  return ret_val;
}

int Driver::I2CWriteReg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  int ret;
  uint8_t wbuf[2];
  uint16_t bytesXfered;

  wbuf[0] = reg_addr;
  wbuf[1] = data;

  ret = ftdi.i2c_TransmitX(1, 1, i2c_addr, wbuf, 2, bytesXfered );
  if (ret != FtdiDriver::I2C_RETURN_CODE_success) {
    fprintf(stderr, "I2CWriteReg reg %x failed %d\n", reg_addr, ret);
    return ret;
  } else if (bytesXfered != 2) {
    fprintf(stderr, "I2CWriteReg reg %x nack b %d\n", reg_addr, bytesXfered);
    return FtdiDriver::I2C_RETURN_CODE_nack;
  }
  //printf("I2CWriteReg %x %x with %x OK\n", i2c_addr, reg_addr, data);

  return FtdiDriver::I2C_RETURN_CODE_success;
}

int Driver::I2CReadReg(uint8_t i2c_addr, uint8_t reg_addr,
                          size_t len, uint8_t* data) {
  int ret;
  uint8_t wbuf[1];
  uint16_t bytesXfered;

  wbuf[0] = reg_addr;

  ret = ftdi.i2c_TransmitX(1, 0, i2c_addr, wbuf, 1, bytesXfered);
  if (ret != FtdiDriver::I2C_RETURN_CODE_success) {
    fprintf(stderr, "I2CReadReg wr reg %x failed %d\n", reg_addr, ret);
    return ret;
  } else if (bytesXfered != 1) {
    fprintf(stderr, "I2CReadReg wr reg %x nack b %d\n", reg_addr, bytesXfered);
    return FtdiDriver::I2C_RETURN_CODE_nack;
  }

  ret = ftdi.i2c_ReceiveX(1, 1, i2c_addr, data, len, bytesXfered);
  if (ret != FtdiDriver::I2C_RETURN_CODE_success) {
    fprintf(stderr, "I2CReadReg rd reg %x failed %d\n", reg_addr, ret);
    return ret;
  } else if (bytesXfered != len) {
    fprintf(stderr, "I2CReadReg rd reg %x nack b %d\n", reg_addr, bytesXfered);
    return FtdiDriver::I2C_RETURN_CODE_nack;
  }

  if (len == 2) {
    uint16_t d = (((uint16_t)data[0]) << 4) | (((uint16_t)data[1]) >> 4);
    data[0] = d & 0xFF;
    data[1] = (d >> 8) & 0xFF;
  }

  return FtdiDriver::I2C_RETURN_CODE_success;
}

int Driver::PMBusWrite(uint8_t cmd, size_t len, uint8_t* data) {
  int ret_val;
  uint8_t wbuf[len+1];
  uint16_t bytesXfered;

  wbuf[0] = cmd;
  for (size_t i = 1; i <= len; i++) {
    wbuf[i] = data[i-1];
  }

  len += 1;
  ret_val = ftdi.i2c_TransmitX(1, 1, IR_I2C_ADDR, wbuf, len, bytesXfered);
  if (ret_val != 0) {
    fprintf(stderr, "PMBus write command failed, %d\n",
	    ret_val);
    return 1;
  }

  if (len != size_t(bytesXfered)) {
    fprintf(stderr, "PMBus write command, %d bytes transfered\n",
	    bytesXfered);
    return 1;
  }

  return 0;
}

int Driver::PMBusRead(uint8_t cmd, size_t len, uint8_t* data) {
  int ret_val;
  uint16_t bytesXfered;

  ret_val = ftdi.i2c_TransmitX(1, 0, IR_I2C_ADDR, &cmd, 1, bytesXfered);
  if (ret_val != 0) {
    fprintf(stderr, "PMBus read command write phase failed, %d\n",
	    ret_val);
    return 1;
  }

  if (1 != size_t(bytesXfered)) {
    fprintf(stderr, "PMBus read command write phase %d bytes transfered\n",
	    bytesXfered);
    return 1;
  }

  ret_val = ftdi.i2c_ReceiveX(1, 1, IR_I2C_ADDR, data, len, bytesXfered);
  if (ret_val != 0) {
    fprintf(stderr, "PMBus read command read phase failed, %d\n",
	    ret_val);
    return 1;
  }

  if (len != size_t(bytesXfered)) {
    fprintf(stderr, "PMBus read command read phase returned %d bytes\n",
	    bytesXfered);
    return 1;
  }

  return 0;
}

double Driver::GetBoardVoltage() {
  switch (vr) {
  case VR_MAX20499:
    {
      uint8_t vid;
      int ret_val = I2CReadReg(VR_I2C_ADDR, 0x7, 1, &vid);
      if (ret_val != 0) {
	fprintf(stderr, "GetBoardVoltage failed to read reg, %d\n", ret_val);
	return 0.0;
      }

      double vr_factor = 100000.0;
      double vr_slope = 625.0;
      double vr_intercept = 24375.0;
      double voltage = (vr_intercept + (vid * vr_slope)) / vr_factor;
      return voltage;
    }

  case VR_IR38263:
    {
      size_t len = 2;
      uint8_t buf[len];
      const uint8_t IR_VOUT_COMMAND = 0x21;

      int ret_val = PMBusRead(IR_VOUT_COMMAND, len, buf);
      if (ret_val != 0) {
	fprintf(stderr, "GetBoardVoltage failed IR_VOUT_COMMAND write, %d\n",
		ret_val);
	return 0.0;
      }

      int code = buf[1] << 8;
      code += buf[0];
      return double(code) / 256.0;
    }

  default:
    fprintf(stderr, "GetBoardVoltage failed due to unspecified voltage regulator\n");
    return 0.0;
  }
}

int Driver::SetBoardVoltage(double voltage) {
  switch (vr) {
  case VR_MAX20499:
    {
      uint8_t vid;
      double vr_factor = 100000.0;
      uint32_t vr_slope = 625;
      uint32_t vr_intercept = 24375;
      vid =
	(uint8_t)((((uint32_t)(voltage * vr_factor)) - vr_intercept) / vr_slope);

      if ((vid < 0x29) || (vid > 0x79)) {
	fprintf(stderr, "SetBoardVoltage illegal vid %d for voltage %1.3f\n",
		vid, voltage);
	return 1;
      }

      int ret_val = I2CWriteReg(VR_I2C_ADDR, 0x7, vid);
      if (ret_val != 0) {
	fprintf(stderr, "SetBoardVoltage failed to write VID reg, %d\n", ret_val);
	return 1;
      }

      return 0;
    }

  case VR_IR38263:
    {
      uint16_t vid;
      vid = uint16_t(voltage * 256.0);

      size_t len = 2;
      uint8_t buf[len];
      const uint8_t IR_VOUT_COMMAND = 0x21;

      buf[0] = vid & 0xff;
      buf[1] = (vid >> 8) & 0xff;

      int ret_val = PMBusWrite(IR_VOUT_COMMAND, len, buf);
      if (ret_val != 0) {
	fprintf(stderr, "SetBoardVoltage failed IR_VOUT_COMMAND write, %d\n",
		ret_val);
	return 1;
      }
      usleep(500000);

      return ret_val;
    }

  default:
    fprintf(stderr, "SetBoardVoltage failed due to unspecified voltage regulator\n");
    return 1;
  }
}

double Driver::GetBoardCurrent() {
  switch (vr) {
  case VR_MAX20499:
    {
      uint16_t cs;

      int ret_val = I2CWriteReg(CS_I2C_ADDR, 0xA, 0x2);
      if (ret_val != 0) {
	fprintf(stderr, "GetBoardCurrent failed to write reg, %d\n", ret_val);
	return 0.0;
      }

      usleep(10000);

      ret_val = I2CReadReg(CS_I2C_ADDR, 0x0, 2, (uint8_t*)(&cs));
      if (ret_val != 0) {
	fprintf(stderr, "GetBoardCurrent failed to read reg, %d\n", ret_val);
	return 0.0;
      }

      double cs_vmax = 440.0; // mv
      double cs_gain = 8.0;
      double cs_adc_max = 4096.0;
      double cs_resistor = 3.0; // mohm

      double c = ((double)cs * cs_vmax) / (cs_gain * cs_adc_max * cs_resistor);

      return c;
    }

  case VR_IR38263:
    {
      size_t len = 2;
      uint8_t buf[len];
      const uint8_t IR_READ_IOUT = 0x8c;

      int ret_val = PMBusRead(IR_READ_IOUT, len, buf);
      if (ret_val != 0) {
	fprintf(stderr, "GetBoardCurrent failed IR_READ_IOUT read, %d\n",
		ret_val);
	return 0.0;
      }

      int code = buf[1] << 8;
      code += buf[0];
      code &= 0x7ff; // 11 bits
      return double(code) / 16.0;;
    }

  default:
    fprintf(stderr, "GetBoardCurrent failed due to unspecified voltage regulator\n");
    return 0.0;
  }
}

double Driver::GetPower() {
  double current = GetBoardCurrent();
  double voltage = GetBoardVoltage();
  return (current * voltage);
}

int Driver::AutoConfigVr() {

  // IR38263
  const uint8_t IR_CLEAR_FAULTS           = 0x03;
  const uint8_t IR_VOUT_OV_FAULT_LIMIT    = 0x40;
  const uint8_t IR_VOUT_OV_FAULT_RESPONSE = 0x41;
  const uint8_t IR_IOUT_OC_FAULT_LIMIT    = 0x46;
  const uint8_t IR_IOUT_OC_FAULT_RESPONSE = 0x47;
  const uint8_t IR_OT_FAULT_LIMIT         = 0x4f;
  const uint8_t IR_OT_FAULT_RESPONSE      = 0x50;
  const uint8_t IR_VIN_OV_FAULT_LIMIT     = 0x55;
  const uint8_t IR_VIN_OV_FAULT_RESPONSE  = 0x56;
  const uint8_t IR_MFR_MODEL              = 0x9a;
  const uint8_t IR_MFR_WRITE_REG          = 0xd1;

  // MAX20499
  const uint8_t VR_ID                     = 0x00;
  const uint8_t VR_COMP                   = 0x08;
  
  vr = VR_none;

  int ret_val;

  // Query I2C to see if IR38263 is attached.
  uint8_t buf[4];
  size_t len;

  len = 4;
  ret_val = PMBusRead(IR_MFR_MODEL, len, buf);
  if (ret_val != 0) {
    fprintf(stderr, "AutoConfigVr failed IR_MFR_MODEL read, %d\n",
	    ret_val);
    return 0.0;
  }

  if (buf[0] == 0x03 && buf[1] == 0x65 && buf[2] == 0x00 && buf[3] == 0x00) {
    vr = VR_IR38263;

    // Set configuration to PMBus mode.
    buf[0] = 0x75; // SVID_PVID_Mode register
    buf[1] = 0x80; // bit 6 1=PVID mode, 0=PMBus mode
    len = 2;

    ret_val = PMBusWrite(IR_MFR_WRITE_REG, len, buf);
    if (ret_val != 0) {
      fprintf(stderr, "AutoConfigVr failed IR_MFR_WRITE_REG write, %d\n",
	      ret_val);
      return 1;
    }

    int vin_ov = int(20.0 * 4.0);    // 20 V
    uint8_t vin_ov_response = 0x80;  // shutdown
    int vout_ov = int(1.2 * 256.0);  // 1.2 V
    uint8_t vout_ov_response = 0x80; // shutdown
    int iout_oc = int(30.0 * 2.0);   // 1.2 V
    uint8_t iout_oc_response = 0xb8; // pulse by pulse for 8 cycles...
    int ot = 125;                    // 125 C
    uint8_t ot_response = 0x80;      // shutdown

    // Program limits.
    buf[0] = uint8_t(vin_ov & 0xff);
    buf[1] = uint8_t((vin_ov >> 8) | 0xf0);
    PMBusWrite(IR_VIN_OV_FAULT_LIMIT, 2, buf);

    buf[0] = uint8_t(vout_ov & 0xff);
    buf[1] = uint8_t(vout_ov >> 8);
    PMBusWrite(IR_VOUT_OV_FAULT_LIMIT, 2, buf);

    buf[0] = uint8_t(iout_oc & 0xff);
    buf[1] = uint8_t((iout_oc >> 8) | 0xf8);
    PMBusWrite(IR_IOUT_OC_FAULT_LIMIT, 2, buf);

    buf[0] = uint8_t(ot & 0xff);
    buf[1] = uint8_t(ot >> 8);
    PMBusWrite(IR_OT_FAULT_LIMIT, 2, buf);

    // Program responses.
    PMBusWrite(IR_VIN_OV_FAULT_RESPONSE,  1, &vin_ov_response);
    PMBusWrite(IR_VOUT_OV_FAULT_RESPONSE, 1, &vout_ov_response);
    PMBusWrite(IR_IOUT_OC_FAULT_RESPONSE, 1, &iout_oc_response);
    PMBusWrite(IR_OT_FAULT_RESPONSE,      1, &ot_response);

    // Clear faults.
    PMBusWrite(IR_CLEAR_FAULTS, 0, buf);

    return 0;
  }

  // Query I2C to see if MAX20499 is attached.
  len = 1;
  ret_val = I2CReadReg(VR_I2C_ADDR, VR_ID, len, buf);
  if (ret_val != 0) {
    fprintf(stderr, "AutoConfigVr failed read VR_ID register, %d\n",
	    ret_val); 
    return 1;
  }

  if ((buf[0] >> 4) == 0x07) { // compare to "DEV" field
    vr = VR_MAX20499;

    ret_val = I2CWriteReg(VR_I2C_ADDR, VR_COMP, 0xe4);
    if (ret_val != 0) {
      fprintf(stderr, "AutoConfigVr failed to write COMP register, %d\n",
	      ret_val);
      return 1;
    }

    return 0;
  }

  fprintf(stderr, "AutoConfigVr failed, no voltage regulator found\n");
  return 1;
}
