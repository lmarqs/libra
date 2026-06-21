#pragma once

// Pure, host-tested clamp for the bench / calibration throttle path (no Arduino deps).
//
// The balancing path is capped at kMaxThrottle inside the mixer. The bench commands drive
// the ESCs directly and bypass that cap, so this is the single place that decides how high
// a *manual* throttle may go: clamped to `cap` (the safety ceiling) unless the operator
// explicitly overrides it for props-off calibration, in which case it may run the full
// 0..1 ESC range. Negative requests floor at 0; the override never exceeds full scale.
namespace throttle {

namespace detail {
constexpr float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace detail

// Single return-statement so it is a valid C++11 constexpr (the embedded toolchain),
// not just under the native test env's gnu++17.
constexpr float clampBenchThrottle(float requested, bool allow_over_cap, float cap) {
  return detail::clampf(requested, 0.0f, allow_over_cap ? 1.0f : cap);
}

}  // namespace throttle
