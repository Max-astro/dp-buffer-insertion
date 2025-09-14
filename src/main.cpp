#include "buffering.h"

int main(int argc, char **argv) {
  // std::cout << "Hello, World!" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  BufInvLib lib;
  BufLibCell &defaultBuf = lib.bufs_[2]; // Use a medium size buffer

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  printf("Time taken to load library: %f seconds\n",
         duration.count() / 1000000.0);

  NetData net = NetData::GenRandomNet(100); // unbalanced
  NodeMgr nodeMgr(net.sinks_.size() * 10);
  ClusterSolver solver(nodeMgr, net, defaultBuf);
  BufNode *src = solver.BuildBufferTree();
  src->EmitDOT("unbalanced.dot");

  {
    DpSolver dpSolver(nodeMgr, src, lib);
    dpSolver.Solve();
    auto *bestSolution = dpSolver.GetBestSolution();
    bestSolution->EmitDOT("unbalanced_dp.dot");
  }

  {
    NetData net = NetData::GenRandomNet(100, 1.0f, 1.2f); // balanced
    ClusterSolver solver(nodeMgr, net, defaultBuf);
    BufNode *src = solver.BuildBufferTree();
    src->EmitDOT("balanced.dot");

    DpSolver dpSolver(nodeMgr, src, lib);
    dpSolver.Solve();
    auto *bestSolution = dpSolver.GetBestSolution();
    bestSolution->EmitDOT("balanced_dp.dot");
  }

  return 0;
}