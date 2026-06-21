#pragma once

// Pure, host-tested clamp for the bench / manual-drive throttle path (no Arduino deps).
//
// The bench commands drive each ESC directly with a raw per-motor throttle, so each value is
// clamped to the full valid ESC range [0, 1] here (the balancing path is bounded separately by
// the mixer's kMaxThrottle). Full range is intentional: ESC throttle-range calibration needs
// the operator to command 1.0 (max) then 0.0 (min) by hand.
namespace throttle {

// Single return-statement so it is a valid C++11 constexpr (the embedded toolchain),
// not just under the native test env's gnu++17.
constexpr float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

}  // namespace throttle
