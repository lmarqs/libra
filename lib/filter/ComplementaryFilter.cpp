#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(float alpha) : _alpha(alpha) {}

void ComplementaryFilter::reset(float angle) {
  _angle = angle;
  _seeded = true;
}

float ComplementaryFilter::update(float accel_angle, float rate, float dt) {
  if (!_seeded) {
    reset(accel_angle);
    return _angle;
  }
  const float predicted = _angle + rate * dt;
  _angle = _alpha * predicted + (1.0f - _alpha) * accel_angle;
  return _angle;
}
