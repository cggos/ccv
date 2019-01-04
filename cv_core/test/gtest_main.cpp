#include "foo_environment.h"

FooEnvironment* foo_env = nullptr;

GTEST_API_ int main(int argc, char **argv) {
  foo_env = new FooEnvironment;
  testing::AddGlobalTestEnvironment(foo_env);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
