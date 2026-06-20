#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(float alpha) : alpha_(alpha) {}

void ComplementaryFilter::reset(float angle) {
  angle_ = angle;
  seeded_ = true;
}

float ComplementaryFilter::update(float accel_angle, float rate, float dt) {
  if (!seeded_) {
    reset(accel_angle);
    return angle_;
  }
  const float predicted = angle_ + rate * dt;
  angle_ = alpha_ * predicted + (1.0f - alpha_) * accel_angle;
  return angle_;
}
