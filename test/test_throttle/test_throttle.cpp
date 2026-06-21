#include <Throttle.h>
#include <unity.h>

using throttle::clampBenchThrottle;

namespace {
constexpr float kCap = 0.05f;  // mirrors config::kMaxThrottle
}

// Within the cap, the request passes through untouched.
void test_in_range_passes_through(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.03f, clampBenchThrottle(0.03f, false, kCap));
}

// Without the override, a request above the cap clamps to the cap.
void test_above_cap_clamps_without_override(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kCap, clampBenchThrottle(0.8f, false, kCap));
}

// With the override, the request may exceed the cap (props-off calibration).
void test_override_allows_above_cap(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, clampBenchThrottle(0.8f, true, kCap));
}

// Even the override is bounded to the top of the ESC range (1.0).
void test_override_clamps_to_full(void) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, clampBenchThrottle(1.5f, true, kCap)); }

// Negative requests floor at zero, with or without the override.
void test_negative_floors_to_zero(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, clampBenchThrottle(-0.2f, false, kCap));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, clampBenchThrottle(-0.2f, true, kCap));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_in_range_passes_through);
  RUN_TEST(test_above_cap_clamps_without_override);
  RUN_TEST(test_override_allows_above_cap);
  RUN_TEST(test_override_clamps_to_full);
  RUN_TEST(test_negative_floors_to_zero);
  return UNITY_END();
}
