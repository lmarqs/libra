#include <ComplementaryFilter.h>
#include <unity.h>

// The first sample seeds the estimate straight to the accel angle, ignoring the
// gyro term — so the filter starts at the true angle instead of ramping from 0.
void test_first_update_seeds_to_accel(void) {
  ComplementaryFilter f(0.98f);
  float a = f.update(/*accel_angle=*/12.5f, /*rate=*/1000.0f, /*dt=*/0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.5f, a);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.5f, f.angle());
}

// alpha = 1 trusts the gyro completely: the estimate integrates rate * dt and
// ignores the accelerometer.
void test_alpha_one_is_pure_gyro_integration(void) {
  ComplementaryFilter f(1.0f);
  f.reset(0.0f);
  for (int i = 0; i < 10; ++i) {
    f.update(/*accel_angle=*/999.0f, /*rate=*/10.0f, /*dt=*/0.1f);
  }
  // 10 deg/s for 10 * 0.1 s = 10 deg, regardless of the accel input.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, f.angle());
}

// alpha = 0 trusts the accelerometer completely: the estimate tracks the accel
// angle exactly and the gyro is ignored.
void test_alpha_zero_is_pure_accel(void) {
  ComplementaryFilter f(0.0f);
  f.reset(0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 7.0f, f.update(7.0f, 5000.0f, 0.02f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -3.0f, f.update(-3.0f, -5000.0f, 0.02f));
}

// With a steady accel angle and no rotation, a wrong initial estimate decays
// toward the accel angle — this is the drift correction the accel provides.
void test_accel_corrects_drift(void) {
  ComplementaryFilter f(0.9f);
  f.reset(20.0f);  // start 20 deg off from the true (accel) angle of 0
  for (int i = 0; i < 100; ++i) {
    f.update(/*accel_angle=*/0.0f, /*rate=*/0.0f, /*dt=*/0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, f.angle());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_first_update_seeds_to_accel);
  RUN_TEST(test_alpha_one_is_pure_gyro_integration);
  RUN_TEST(test_alpha_zero_is_pure_accel);
  RUN_TEST(test_accel_corrects_drift);
  return UNITY_END();
}
