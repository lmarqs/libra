#pragma once

#include <cstdint>

// One fused reading from the MPU6050, in physical units.
struct ImuSample {
  float ax, ay, az;  // acceleration, g
  float gx, gy, gz;  // angular rate, deg/s
};

// Minimal MPU6050 driver over I2C (Wire). Reads raw accel + gyro registers and
// converts to physical units at the default full-scale ranges (±2 g, ±250 °/s).
// Gyro bias is sampled at startup and subtracted, so the beam must be held still
// during begin().
//
// Hardware-only: depends on Arduino/Wire, so it is never built by the native
// test env (the angle-fusion math it feeds lives in the host-tested
// ComplementaryFilter instead).
class Imu {
 public:
  explicit Imu(uint8_t address);

  // Wake the device, configure the low-pass filter, then calibrate the gyro
  // bias (keep the sensor still). Returns false if the MPU6050 doesn't ACK.
  bool begin();

  // Read one sample, converted to g / deg/s with the gyro bias removed.
  // Returns false on an I2C error.
  bool read(ImuSample& out);

  // Tilt angle (deg) about the pivot axis, derived from gravity alone.
  static float accelAngleDeg(const ImuSample& s);

 private:
  bool writeReg(uint8_t reg, uint8_t value);
  bool readRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
  void calibrateGyro(uint16_t samples);

  uint8_t _address;
  float _gx_bias = 0.0f;
  float _gy_bias = 0.0f;
  float _gz_bias = 0.0f;
};
