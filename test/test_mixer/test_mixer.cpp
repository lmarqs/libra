#include <Mixer.h>
#include <unity.h>

// No correction -> both motors sit at the base throttle.
void test_no_correction_is_base(void) {
  Mixer m(0.5f);
  MotorCommands c = m.mix(0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, c.m1);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, c.m2);
}

// A positive correction speeds up motor 1 and slows motor 2 by the same amount
// (the differential thrust that rotates the beam).
void test_correction_is_differential(void) {
  Mixer m(0.5f);
  MotorCommands c = m.mix(0.2f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.7f, c.m1);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, c.m2);
}

// Commands clamp to [min, max] so a big correction can't go out of range.
void test_clamps_to_range(void) {
  Mixer m(0.8f, 0.0f, 1.0f);
  MotorCommands hi = m.mix(0.5f);  // 0.8 + 0.5 = 1.3 -> 1.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, hi.m1);
  Mixer m2(0.2f, 0.0f, 1.0f);
  MotorCommands lo = m2.mix(0.5f);  // 0.2 - 0.5 = -0.3 -> 0.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, lo.m2);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_no_correction_is_base);
  RUN_TEST(test_correction_is_differential);
  RUN_TEST(test_clamps_to_range);
  return UNITY_END();
}
