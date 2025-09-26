#include "buffering.h"

void Benchmark(const Sky130Lib &lib) {
  const OTTimingArc &defaultBuf = lib.bufs_[2]; // Use a medium size buffer
  // size_t FANOUTS[] = {100,  500,  1000, 2000, 3000, 3500,
  //                     4000, 5000, 6000, 7000, 8000};

  size_t FANOUTS[] = {100, 120, 150, 200};
  for (size_t fanouts : FANOUTS) {

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

    std::string fname = "dp_" + std::to_string(fanouts) + ".dot";
    bestSolution->EmitDOT(fname.c_str());
  }
}

int main(int argc, char **argv) {
  auto start = std::chrono::high_resolution_clock::now();

  ot::Timer timer;
  Sky130Lib::InitMockTimer(timer);
  Sky130Lib lib(timer);
  OTTimingArc &defaultBuf = lib.bufs_[2]; // Use a medium size buffer

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  printf("Time taken to load library: %f seconds\n",
         duration.count() / 1000000.0);

  Benchmark(lib);
  return 0;
}