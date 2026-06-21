#include <Throttle.h>
#include <unity.h>

using throttle::clamp01;

// In-range values pass through untouched.
void test_in_range_passes_through(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.03f, clamp01(0.03f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, clamp01(0.8f));
}

// Above full scale clamps to 1.0.
void test_above_one_clamps(void) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, clamp01(1.5f)); }

// Below zero clamps to 0.0.
void test_below_zero_clamps(void) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, clamp01(-0.2f)); }

// The boundaries are preserved exactly.
void test_boundaries(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, clamp01(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, clamp01(1.0f));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_in_range_passes_through);
  RUN_TEST(test_above_one_clamps);
  RUN_TEST(test_below_zero_clamps);
  RUN_TEST(test_boundaries);
  return UNITY_END();
}
