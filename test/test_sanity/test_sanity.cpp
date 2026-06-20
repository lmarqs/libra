// Smallest possible host test — proves the `native` env + Unity harness build
// and run. Real control-logic tests (pid / filter / mixer) replace the need for
// this as those libs land.
#include <unity.h>

void test_sanity(void) { TEST_ASSERT_EQUAL_INT(2, 1 + 1); }

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_sanity);
  return UNITY_END();
}
