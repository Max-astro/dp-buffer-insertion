#include "buffering.h" // Include your header files here
#include <gtest/gtest.h>

// Test fixture class (optional, useful for more complex tests)
class CpuTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // You can declare any variables or helper functions here
};

// Basic test example - Testing the GetBufNodeTypeStr function
TEST(BufferingTest, GetBufNodeTypeStrReturnsCorrectStrings) {
  EXPECT_STREQ(GetBufNodeTypeStr(BufNodeType::Init), "Init");
  EXPECT_STREQ(GetBufNodeTypeStr(BufNodeType::Buffer), "Buffer");
  EXPECT_STREQ(GetBufNodeTypeStr(BufNodeType::Inverter), "Inverter");
  EXPECT_STREQ(GetBufNodeTypeStr(BufNodeType::Removed), "Removed");
  EXPECT_STREQ(GetBufNodeTypeStr(BufNodeType::Sink), "Sink");
  EXPECT_STREQ(GetBufNodeTypeStr(BufNodeType::Src), "Src");
}

// Test using the test fixture
TEST_F(CpuTest, DummyTestExample) {
  // This is a dummy test to demonstrate the test fixture usage
  EXPECT_EQ(2 + 2, 4);
  EXPECT_TRUE(true);
  EXPECT_FALSE(false);
}

// Parameterized test example
class BufNodeTypeParameterizedTest
    : public ::testing::TestWithParam<std::pair<BufNodeType, const char *>> {};

TEST_P(BufNodeTypeParameterizedTest, GetBufNodeTypeStrParameterized) {
  auto [nodeType, expectedStr] = GetParam();
  EXPECT_STREQ(GetBufNodeTypeStr(nodeType), expectedStr);
}

INSTANTIATE_TEST_SUITE_P(
    BufferingParameterizedTests, BufNodeTypeParameterizedTest,
    ::testing::Values(std::make_pair(BufNodeType::Init, "Init"),
                      std::make_pair(BufNodeType::Buffer, "Buffer"),
                      std::make_pair(BufNodeType::Inverter, "Inverter"),
                      std::make_pair(BufNodeType::Removed, "Removed"),
                      std::make_pair(BufNodeType::Sink, "Sink"),
                      std::make_pair(BufNodeType::Src, "Src")));

// Death test example (for testing assertions/crashes)
TEST(BufferingDeathTest, DISABLED_ExampleDeathTest) {
  // This test is disabled by default. Remove DISABLED_ to enable it.
  // EXPECT_DEATH(someFunction(), "Expected error message");
}

// Performance/benchmark-style test example
TEST(BufferingPerformanceTest, DISABLED_ExamplePerformanceTest) {
  // This test is disabled by default. Remove DISABLED_ to enable it.
  auto start = std::chrono::high_resolution_clock::now();

  // Your code to benchmark here
  for (int i = 0; i < 1000000; ++i) {
    GetBufNodeTypeStr(BufNodeType::Buffer);
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  // You can add expectations about performance if needed
  EXPECT_LT(duration.count(), 1000000); // Should take less than 1 second
}
