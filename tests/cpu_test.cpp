#include "buffering.h" // Include your header files here
#include <cmath>
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

TEST_F(CpuTest, ParetoFrontierTest_InCap) {
  NetData net = NetData::GenRandomNet(30);
  BufInvLib lib;

  ClusterSolver solver(nodeMgr_, net, lib_.bufs_[2]);
  BufNode *src = solver.BuildBufferTree();

  DpSolver dpSolver(nodeMgr_, src, lib, lib_.bufs_[2]);

  // simple test
  BufNodeRbTree rbt;

  auto *p1 = nodeMgr_.Alloc();
  p1->ty_ = BufNodeType::Buffer;
  p1->rat_ = 10.0;
  p1->inCap_ = 0.1;
  dpSolver.MaintainFrontier(p1, rbt);

  EXPECT_EQ(rbt.size(), 1);

  // p2 is dominated by p1
  auto *p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Buffer;
  p->rat_ = 9.0;
  p->inCap_ = 0.2;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 1);
  EXPECT_EQ((*rbt.begin())->uid_, p1->uid_);

  // new element in the frontier
  p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Buffer;
  p->rat_ = 9.0;
  p->inCap_ = 0.05;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 2);

  // new element in the frontier
  p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Buffer;
  p->rat_ = 12.0;
  p->inCap_ = 0.35;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 3);

  // new element dominate all existing elements
  p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Buffer;
  p->rat_ = 15.0;
  p->inCap_ = 0.01;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 1);
  EXPECT_EQ((*rbt.begin())->uid_, p->uid_);

  // similar element
  p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Buffer;
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

TEST_F(CpuTest, ParetoFrontierTest_RandomInCap) {
  NetData net = NetData::GenRandomNet(1000);
  BufInvLib lib;

  ClusterSolver solver(nodeMgr_, net, lib_.bufs_[2]);
  BufNode *src = solver.BuildBufferTree();

  DpSolver dpSolver(nodeMgr_, src, lib, lib_.bufs_[2]);

  // src->ty_ = BufNodeType::Buffer; // hack for testing
  // simple test
  BufNodeRbTree rbt;
  for (auto *node : src->TopologicalSort()) {
    node->loading_ = 0.1; // hack for testing
    dpSolver.MaintainFrontier(node, rbt);
  }

  EXPECT_TRUE(CheckParetoFrontier(dpSolver, rbt));
}

TEST_F(CpuTest, ParetoFrontierTest_Loading) {
  NetData net = NetData::GenRandomNet(30);
  BufInvLib lib;

  ClusterSolver solver(nodeMgr_, net, lib_.bufs_[2]);
  BufNode *src = solver.BuildBufferTree();

  DpSolver dpSolver(nodeMgr_, src, lib, lib_.bufs_[2]);

  // simple test
  BufNodeRbTree rbt;

  auto *p1 = nodeMgr_.Alloc();
  p1->ty_ = BufNodeType::Init;
  p1->rat_ = 10.0;
  p1->loading_ = 0.1;
  dpSolver.MaintainFrontier(p1, rbt);

  EXPECT_EQ(rbt.size(), 1);

  // p2 is dominated by p1
  auto *p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Init;
  p->rat_ = 9.0;
  p->loading_ = 0.2;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 1);
  EXPECT_EQ((*rbt.begin())->uid_, p1->uid_);

  // new element in the frontier
  p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Init;
  p->rat_ = 9.0;
  p->loading_ = 0.05;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 2);

  // new element in the frontier
  p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Init;
  p->rat_ = 12.0;
  p->loading_ = 0.35;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 3);

  // new element dominate all existing elements
  p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Init;
  p->rat_ = 15.0;
  p->loading_ = 0.01;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 1);
  EXPECT_EQ((*rbt.begin())->uid_, p->uid_);

  // similar element
  p = nodeMgr_.Alloc();
  p->ty_ = BufNodeType::Init;
  p->rat_ = 15.001;
  p->loading_ = 0.0101;
  dpSolver.MaintainFrontier(p, rbt);

  EXPECT_EQ(rbt.size(), 1);
  EXPECT_EQ((*rbt.begin())->uid_, p->uid_);
}

TEST_F(CpuTest, ParetoFrontierTest_RandomLoading) {
  NetData net = NetData::GenRandomNet(1000);
  BufInvLib lib;

  ClusterSolver solver(nodeMgr_, net, lib_.bufs_[2]);
  BufNode *src = solver.BuildBufferTree();

  DpSolver dpSolver(nodeMgr_, src, lib, lib_.bufs_[2]);

  // src->ty_ = BufNodeType::Buffer; // hack for testing
  // simple test
  BufNodeRbTree rbt;
  for (auto *node : src->TopologicalSort()) {
    node->ty_ = BufNodeType::Init;
    node->loading_ = node->inCap_; // hack for testing
    node->inCap_ = 0.0;            // hack for testing
    dpSolver.MaintainFrontier(node, rbt);
  }

  EXPECT_TRUE(CheckParetoFrontier(dpSolver, rbt));
}

TEST_F(CpuTest, CalcDelay_MonotonicWithLoading_Buffer) {
  const BufLibCell &cell = lib_.bufs_[2];
  const double tr = BufInvLib::DEFAULT_TRANS;

  const double l1 = 0.002;
  const double l2 = 0.01;
  const double l3 = 0.05;

  const double r1 = cell.CalcDelay(DelayType::Rise, tr, l1);
  const double r2 = cell.CalcDelay(DelayType::Rise, tr, l2);
  const double r3 = cell.CalcDelay(DelayType::Rise, tr, l3);

  const double f1 = cell.CalcDelay(DelayType::Fall, tr, l1);
  const double f2 = cell.CalcDelay(DelayType::Fall, tr, l2);
  const double f3 = cell.CalcDelay(DelayType::Fall, tr, l3);

  EXPECT_LE(r1, r2);
  EXPECT_LE(r2, r3);
  EXPECT_LE(f1, f2);
  EXPECT_LE(f2, f3);

  // expected values
  float vr1 = 0.076293, vr2 = 0.095338, vr3 = 0.173246, vf1 = 0.107588,
        vf2 = 0.123129, vf3 = 0.170985;

  EXPECT_NEAR(r1, vr1, 1e-6);
  EXPECT_NEAR(r2, vr2, 1e-6);
  EXPECT_NEAR(r3, vr3, 1e-6);
  EXPECT_NEAR(f1, vf1, 1e-6);
  EXPECT_NEAR(f2, vf2, 1e-6);
  EXPECT_NEAR(f3, vf3, 1e-6);
}

TEST_F(CpuTest, CalcDelay_sky130) {
  const BufLibCell &cell = lib_.bufs_[2];
  const double load = 0.02;

  const double t1 = 0.005;
  const double t2 = 0.02;
  const double t3 = 0.08;

  const double r1 = cell.CalcDelay(DelayType::Rise, t1, load);
  const double r2 = cell.CalcDelay(DelayType::Rise, t2, load);
  const double r3 = cell.CalcDelay(DelayType::Rise, t3, load);

  const double f1 = cell.CalcDelay(DelayType::Fall, t1, load);
  const double f2 = cell.CalcDelay(DelayType::Fall, t2, load);
  const double f3 = cell.CalcDelay(DelayType::Fall, t3, load);

  EXPECT_LE(r1, r2);
  EXPECT_LE(r2, r3);
  EXPECT_LE(f1, f2);
  EXPECT_LE(f2, f3);
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
