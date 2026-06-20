#include "Mixer.h"

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

Mixer::Mixer(float base, float min_out, float max_out) : base_(base), min_out_(min_out), max_out_(max_out) {}

MotorCommands Mixer::mix(float correction) const {
  return {clampf(base_ + correction, min_out_, max_out_), clampf(base_ - correction, min_out_, max_out_)};
}
