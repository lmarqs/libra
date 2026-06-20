#include <Pid.h>
#include <unity.h>

// No error and no accumulated state -> no output.
void test_zero_error_zero_output(void) {
  Pid pid({1.0f, 0.0f, 0.0f}, -10.0f, 10.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, pid.update(0.0f, 0.0f, 0.01f));
}

// Pure proportional: output = kp * (setpoint - measurement).
void test_proportional_only(void) {
  Pid pid({2.0f, 0.0f, 0.0f}, -100.0f, 100.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.0f, pid.update(/*sp=*/5.0f, /*meas=*/2.0f, 0.01f));
}

// Pure integral accumulates error*dt over time and scales by ki.
void test_integral_accumulates(void) {
  Pid pid({0.0f, 1.0f, 0.0f}, -100.0f, 100.0f);
  float out = 0.0f;
  for (int i = 0; i < 10; ++i) out = pid.update(/*sp=*/1.0f, /*meas=*/0.0f, 0.1f);
  // error 1 for 10 * 0.1 s -> integral 1.0, ki 1.0 -> output 1.0.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, out);
}

// Output saturates at the configured limits.
void test_output_clamped(void) {
  Pid pid({100.0f, 0.0f, 0.0f}, -1.0f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, pid.update(10.0f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, pid.update(-10.0f, 0.0f, 0.01f));
}

// Anti-windup: a long saturating error must not let the integrator grow past
// what the output range allows, so recovery is immediate when error reverses.
void test_anti_windup(void) {
  Pid pid({0.0f, 1.0f, 0.0f}, -1.0f, 1.0f);
  for (int i = 0; i < 1000; ++i) pid.update(/*sp=*/100.0f, /*meas=*/0.0f, 0.1f);
  // Integral contribution is capped at out_max/ki = 1.0, not the ~10000 raw sum.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, pid.integrator());
  // One step with the error reversed drops the output below the limit at once.
  TEST_ASSERT_TRUE(pid.update(/*sp=*/0.0f, /*meas=*/100.0f, 0.1f) < 1.0f);
}

// Derivative acts on measurement, so a setpoint step produces no derivative
// kick (output stays purely proportional on the step).
void test_derivative_on_measurement_no_kick(void) {
  Pid pid({1.0f, 0.0f, 5.0f}, -1000.0f, 1000.0f);
  pid.update(/*sp=*/0.0f, /*meas=*/0.0f, 0.01f);  // seed derivative history
  // Setpoint jumps to 10 but measurement is unchanged -> derivative term 0.
  float out = pid.update(/*sp=*/10.0f, /*meas=*/0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, out);  // kp*error only
}

// Derivative opposes a rising measurement.
void test_derivative_on_rising_measurement(void) {
  Pid pid({0.0f, 0.0f, 2.0f}, -1000.0f, 1000.0f);
  pid.update(0.0f, 0.0f, 0.1f);                  // seed
  float out = pid.update(0.0f, 1.0f, 0.1f);      // meas rises 0->1 over 0.1s -> rate 10
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -20.0f, out);  // -kd * 10
}

// reset() clears the integrator.
void test_reset_clears_integral(void) {
  Pid pid({0.0f, 1.0f, 0.0f}, -100.0f, 100.0f);
  pid.update(1.0f, 0.0f, 1.0f);
  TEST_ASSERT_TRUE(pid.integrator() > 0.0f);
  pid.reset();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.integrator());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_zero_error_zero_output);
  RUN_TEST(test_proportional_only);
  RUN_TEST(test_integral_accumulates);
  RUN_TEST(test_output_clamped);
  RUN_TEST(test_anti_windup);
  RUN_TEST(test_derivative_on_measurement_no_kick);
  RUN_TEST(test_derivative_on_rising_measurement);
  RUN_TEST(test_reset_clears_integral);
  return UNITY_END();
}
