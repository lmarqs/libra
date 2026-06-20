#pragma once

// Complementary filter for single-axis tilt estimation.
//
// Fuses two imperfect signals into one good angle estimate:
//   - the accelerometer's gravity-derived angle: absolute but noisy and
//     corrupted by linear acceleration,
//   - the gyroscope's angular rate: smooth and fast but drifts when integrated.
//
// Each update blends a gyro-integrated prediction with the accel angle:
//   angle = alpha * (angle + rate*dt) + (1 - alpha) * accel_angle
// A high alpha (~0.98) trusts the gyro short-term and lets the accelerometer
// slowly correct drift. Pure math, no Arduino dependencies — host-testable.
class ComplementaryFilter {
 public:
  // alpha is the gyro weight in [0, 1]; (1 - alpha) goes to the accelerometer.
  explicit ComplementaryFilter(float alpha);

  // Seed the estimate directly (e.g. to the first accel reading) so it doesn't
  // ramp up from zero. The next update() blends from here.
  void reset(float angle);

  // Fuse one sample and return the new estimate.
  //   accel_angle: absolute angle from gravity (same unit as the result, e.g. deg)
  //   rate:        angular rate about the same axis (unit/sec, e.g. deg/s)
  //   dt:          time since the previous sample (seconds)
  // The very first update after construction seeds to accel_angle.
  float update(float accel_angle, float rate, float dt);

  float angle() const { return angle_; }

 private:
  float alpha_;
  float angle_ = 0.0f;
  bool seeded_ = false;
};
