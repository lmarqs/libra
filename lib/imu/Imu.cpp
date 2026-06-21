#include "Imu.h"

#include <Arduino.h>
#include <Wire.h>

#include <cmath>

namespace {
// MPU6050 register map (subset).
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;

// Sensitivities at the default full-scale ranges.
constexpr float kAccelLsbPerG = 16384.0f;  // ±2 g
constexpr float kGyroLsbPerDps = 131.0f;   // ±250 °/s
constexpr float kRadToDeg = 57.2957795131f;
}  // namespace

Imu::Imu(uint8_t address) : _address(address) {}

bool Imu::writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Imu::begin() {
  // Wake from sleep (PWR_MGMT_1 defaults to sleep=1 on power-up). This also
  // doubles as the presence check — a missing device won't ACK.
  if (!writeReg(kRegPwrMgmt1, 0x00)) return false;
  // ~1 kHz sample rate, DLPF ~44 Hz to tame propeller vibration, default ranges.
  writeReg(kRegSmplrtDiv, 0x00);
  writeReg(kRegConfig, 0x03);
  writeReg(kRegGyroConfig, 0x00);   // ±250 °/s
  writeReg(kRegAccelConfig, 0x00);  // ±2 g
  delay(50);
  calibrateGyro(500);
  return true;
}

bool Imu::readRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  Wire.beginTransmission(_address);
  Wire.write(kRegAccelXoutH);
  if (Wire.endTransmission(false) != 0) return false;
  // 14 bytes: accel XYZ, temp, gyro XYZ (each big-endian). Buffer first so the
  // two reads per word are sequenced (the order within `a << 8 | b` isn't).
  if (Wire.requestFrom(_address, static_cast<uint8_t>(14)) != 14) return false;
  uint8_t b[14];
  for (uint8_t i = 0; i < 14; ++i) b[i] = static_cast<uint8_t>(Wire.read());
  auto be16 = [&b](uint8_t hi) { return static_cast<int16_t>((b[hi] << 8) | b[hi + 1]); };
  ax = be16(0);
  ay = be16(2);
  az = be16(4);
  // b[6..7] is temperature — discarded.
  gx = be16(8);
  gy = be16(10);
  gz = be16(12);
  return true;
}

void Imu::calibrateGyro(uint16_t samples) {
  float sx = 0, sy = 0, sz = 0;
  uint16_t taken = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRaw(ax, ay, az, gx, gy, gz)) continue;
    sx += gx;
    sy += gy;
    sz += gz;
    ++taken;
    delay(2);
  }
  if (taken == 0) return;
  _gx_bias = (sx / taken) / kGyroLsbPerDps;
  _gy_bias = (sy / taken) / kGyroLsbPerDps;
  _gz_bias = (sz / taken) / kGyroLsbPerDps;
}

bool Imu::read(ImuSample& out) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!readRaw(ax, ay, az, gx, gy, gz)) return false;
  out.ax = ax / kAccelLsbPerG;
  out.ay = ay / kAccelLsbPerG;
  out.az = az / kAccelLsbPerG;
  out.gx = gx / kGyroLsbPerDps - _gx_bias;
  out.gy = gy / kGyroLsbPerDps - _gy_bias;
  out.gz = gz / kGyroLsbPerDps - _gz_bias;
  return true;
}

float Imu::accelAngleDeg(const ImuSample& s) {
  // Tilt about the Z axis: at rest gravity rests on +X, and rotating the beam
  // about Z swings it into Y, so the tilt is gravity's split between X and Y.
  // Mounting-specific — re-derive with the raw-IMU debug stream if remounted.
  return atan2f(s.ay, s.ax) * kRadToDeg;
}
