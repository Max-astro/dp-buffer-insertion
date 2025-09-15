#include "buffering.h"

void Benchmark(const BufInvLib &lib) {
  const BufLibCell &defaultBuf = lib.bufs_[2]; // Use a medium size buffer
  for (size_t fanouts : {100, 500, 1000, 2000, 3000, 3500, 4000, 5000, 6000,
                         7000, 8000}) {

    NetData net =
        NetData::GenRandomNet(fanouts, 1.0f, 1.2f, 0.001, 0.01); // balanced
    NodeMgr nodeMgr(net.sinks_.size() * 1000);
    ClusterSolver solver(nodeMgr, net, defaultBuf);
    BufNode *src = solver.BuildBufferTree();
    // src->EmitDOT("balanced.dot");

    auto start = std::chrono::high_resolution_clock::now();
    DpSolver dpSolver(nodeMgr, src, lib, defaultBuf);
    dpSolver.Solve();
    auto *bestSolution = dpSolver.GetBestSolution();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("Fanout: %zu, Time taken to DP algorithm: %f seconds\n", fanouts,
           duration.count() / 1000000.0);

    // bestSolution->EmitDOT("balanced_dp.dot");
  }
}

int main(int argc, char **argv) {
  auto start = std::chrono::high_resolution_clock::now();

  BufInvLib lib;
  BufLibCell &defaultBuf = lib.bufs_[2]; // Use a medium size buffer

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  printf("Time taken to load library: %f seconds\n",
         duration.count() / 1000000.0);

  Benchmark(lib);
  return 0;
}