#include "buffering.h" // Include your header files here
#include <gtest/gtest.h>

// Test fixture class (optional, useful for more complex tests)
class CpuTest : public ::testing::Test {
protected:
  NodeMgr nodeMgr_;
  BufInvLib lib_;
  CpuTest() : nodeMgr_(1000), lib_() {}

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
TEST(BufferingTest, DummyTestExampleP) {}

// Test using the test fixture
TEST_F(CpuTest, ParetoFrontierTest_Simple) {
  NetData net = NetData::GenRandomNet(30);
  BufInvLib lib;

  ClusterSolver solver(nodeMgr_, net, lib_.bufs_[2]);
  BufNode *src = solver.BuildBufferTree();

  DpSolver dpSolver(nodeMgr_, src, lib, lib_.bufs_[2]);

  // simple test
  BufNodeRbTree rbt;

  auto *p1 = nodeMgr_.Alloc();
  p1->rat_ = 10.0;
  p1->inCap_ = 0.1;
  dpSolver.MaintainFrontier(p1, rbt);

  EXPECT_EQ(rbt.size(), 1);

  // p2 is dominated by p1
  auto *p = nodeMgr_.Alloc();
  p->rat_ = 9.0;
  p->inCap_ = 0.2;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 1);
  EXPECT_EQ((*rbt.begin())->uid_, p1->uid_);

  // new element in the frontier
  p = nodeMgr_.Alloc();
  p->rat_ = 9.0;
  p->inCap_ = 0.05;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 2);

  // new element in the frontier
  p = nodeMgr_.Alloc();
  p->rat_ = 12.0;
  p->inCap_ = 0.35;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 3);

  // new element dominate all existing elements
  p = nodeMgr_.Alloc();
  p->rat_ = 15.0;
  p->inCap_ = 0.01;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 1);
  EXPECT_EQ((*rbt.begin())->uid_, p->uid_);

  // similar element
  p = nodeMgr_.Alloc();
  p->rat_ = 15.001;
  p->inCap_ = 0.0101;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 1);
  EXPECT_EQ((*rbt.begin())->uid_, p->uid_);
}

bool CheckParetoFrontier(const DpSolver &dpSolver, BufNodeRbTree &rbt) {
  for (auto it = rbt.begin(); it != rbt.end(); ++it) {
    for (auto it2 = std::next(it); it2 != rbt.end(); ++it2) {
      if (dpSolver.CheckDominate(*it, *it2)) {
        return false;
      }
    }
  }
  return true;
}

TEST_F(CpuTest, ParetoFrontierTest_Random) {
  NetData net = NetData::GenRandomNet(1000);
  BufInvLib lib;

  ClusterSolver solver(nodeMgr_, net, lib_.bufs_[2]);
  BufNode *src = solver.BuildBufferTree();

  DpSolver dpSolver(nodeMgr_, src, lib, lib_.bufs_[2]);

  // simple test
  BufNodeRbTree rbt;
  for (auto *node : src->TopologicalSort()) {
    dpSolver.MaintainFrontier(node, rbt);
  }

  EXPECT_TRUE(CheckParetoFrontier(dpSolver, rbt));
}

// // Parameterized test example
// class BufNodeTypeParameterizedTest
//     : public ::testing::TestWithParam<std::pair<BufNodeType, const char *>>
//     {};

// TEST_P(BufNodeTypeParameterizedTest, GetBufNodeTypeStrParameterized) {
//   auto [nodeType, expectedStr] = GetParam();
//   EXPECT_STREQ(GetBufNodeTypeStr(nodeType), expectedStr);
// }

// INSTANTIATE_TEST_SUITE_P(
//     BufferingParameterizedTests, BufNodeTypeParameterizedTest,
//     ::testing::Values(std::make_pair(BufNodeType::Init, "Init"),
//                       std::make_pair(BufNodeType::Buffer, "Buffer"),
//                       std::make_pair(BufNodeType::Inverter, "Inverter"),
//                       std::make_pair(BufNodeType::Removed, "Removed"),
//                       std::make_pair(BufNodeType::Sink, "Sink"),
//                       std::make_pair(BufNodeType::Src, "Src")));

// // Death test example (for testing assertions/crashes)
// TEST(BufferingDeathTest, DISABLED_ExampleDeathTest) {
//   // This test is disabled by default. Remove DISABLED_ to enable it.
//   // EXPECT_DEATH(someFunction(), "Expected error message");
// }

// // Performance/benchmark-style test example
// TEST(BufferingPerformanceTest, DISABLED_ExamplePerformanceTest) {
//   // This test is disabled by default. Remove DISABLED_ to enable it.
//   auto start = std::chrono::high_resolution_clock::now();

//   // Your code to benchmark here
//   for (int i = 0; i < 1000000; ++i) {
//     GetBufNodeTypeStr(BufNodeType::Buffer);
//   }

//   auto end = std::chrono::high_resolution_clock::now();
//   auto duration =
//       std::chrono::duration_cast<std::chrono::microseconds>(end - start);

//   // You can add expectations about performance if needed
//   EXPECT_LT(duration.count(), 1000000); // Should take less than 1 second
// }
