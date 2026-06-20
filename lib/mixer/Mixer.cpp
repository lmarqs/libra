#include "Mixer.h"

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

Mixer::Mixer(float base, float min_out, float max_out) : _base(base), _min_out(min_out), _max_out(max_out) {}

MotorCommands Mixer::mix(float correction) const {
  return {clampf(_base + correction, _min_out, _max_out), clampf(_base - correction, _min_out, _max_out)};
}
