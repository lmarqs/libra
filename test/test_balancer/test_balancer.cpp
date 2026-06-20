#include <Balancer.h>
#include <unity.h>

namespace {
// alpha=0 -> the filter reports the accel angle verbatim, so tilt behaviour is
// deterministic in tests. base throttle 0.3, wide PID limits.
Balancer makeBalancer(Pid::Gains gains, float tilt_limit = 45.0f) {
  return Balancer(ComplementaryFilter(0.0f), Pid(gains, -1.0f, 1.0f), Mixer(0.3f), tilt_limit);
}

ControlInputs at(float angle, bool enabled, float setpoint = 0.0f) {
  return ControlInputs{angle, /*rate=*/0.0f, setpoint, /*dt=*/0.01f, enabled};
}
}  // namespace

// Enabled and level: motors sit at the base throttle, nothing tripped.
void test_enabled_level_holds_base(void) {
  Balancer b = makeBalancer({0.0f, 0.0f, 0.0f});
  ControlOutputs o = b.step(at(0.0f, true));
  TEST_ASSERT_TRUE(o.active);
  TEST_ASSERT_FALSE(o.tripped);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, o.m1);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, o.m2);
}

// Disabled: angle still tracked, but motors idle and nothing is active.
void test_disabled_idles_motors(void) {
  Balancer b = makeBalancer({1.0f, 0.0f, 0.0f});
  ControlOutputs o = b.step(at(10.0f, false));
  TEST_ASSERT_FALSE(o.active);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, o.angle);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.m1);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.m2);
}

// Past the tilt limit the failsafe trips and the motors cut, even though the
// operator asked to be enabled.
void test_tilt_limit_trips_failsafe(void) {
  Balancer b = makeBalancer({1.0f, 0.0f, 0.0f}, /*tilt_limit=*/45.0f);
  ControlOutputs o = b.step(at(60.0f, true));
  TEST_ASSERT_TRUE(o.tripped);
  TEST_ASSERT_FALSE(o.active);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.m1);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.m2);
}

// A correction speeds one motor and slows the other (differential thrust).
void test_correction_is_differential(void) {
  Balancer b = makeBalancer({0.01f, 0.0f, 0.0f});
  ControlOutputs o = b.step(at(/*angle=*/-5.0f, true));  // error = +5 deg
  TEST_ASSERT_TRUE(o.output > 0.0f);
  TEST_ASSERT_TRUE(o.m1 > 0.3f);
  TEST_ASSERT_TRUE(o.m2 < 0.3f);
}

// Re-arming resets the integrator so wind-up from a previous run can't kick.
void test_rearm_resets_integrator(void) {
  Balancer b = makeBalancer({0.0f, 1.0f, 0.0f});  // pure integral
  // Build up integral over several enabled steps at a constant error.
  float wound = 0.0f;
  for (int i = 0; i < 5; ++i) wound = b.step(at(-1.0f, true)).output;
  TEST_ASSERT_TRUE(wound > 0.02f);  // integral has accumulated

  b.step(at(-1.0f, false));                      // disable for a step
  float after = b.step(at(-1.0f, true)).output;  // re-arm
  // After reset the integral reflects a single step (error*dt = 0.01), not the
  // accumulated ~0.05+ — so the re-armed output is far smaller than before.
  TEST_ASSERT_TRUE(after < wound);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.01f, after);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_enabled_level_holds_base);
  RUN_TEST(test_disabled_idles_motors);
  RUN_TEST(test_tilt_limit_trips_failsafe);
  RUN_TEST(test_correction_is_differential);
  RUN_TEST(test_rearm_resets_integrator);
  return UNITY_END();
}
