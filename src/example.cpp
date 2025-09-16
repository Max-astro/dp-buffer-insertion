#include "buffering.h"

void RunExample() {
  BufInvLib lib;
  BufLibCell &defaultBuf = lib.bufs_[2]; // Use a medium size buffer
  {
    NetData net =
        NetData::GenRandomNet(30, 1.0f, 2.0f, 0.001, 0.01); // unbalanced
    NodeMgr nodeMgr(net.sinks_.size() * 100);
    ClusterSolver solver(nodeMgr, net, defaultBuf);
    BufNode *src = solver.BuildBufferTree();
    src->EmitDOT("unbalanced.dot");

    auto start = std::chrono::high_resolution_clock::now();
    DpSolver dpSolver(nodeMgr, src, lib, defaultBuf);
    dpSolver.Solve();
    auto *bestSolution = dpSolver.GetBestSolution();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("Time taken to DP algorithm: %f seconds\n",
           duration.count() / 1000000.0);

    DpSolver::ReportImprovement(net, bestSolution, defaultBuf);
    bestSolution->EmitDOT("result_30.dot");
  }

  {
    NetData net =
        NetData::GenRandomNet(500, 1.0f, 5.0f, 0.001, 0.01); // unbalanced
    NodeMgr nodeMgr(net.sinks_.size() * 100);
    ClusterSolver solver(nodeMgr, net, defaultBuf);
    BufNode *src = solver.BuildBufferTree();
    src->EmitDOT("unbalanced.dot");

    auto start = std::chrono::high_resolution_clock::now();
    DpSolver dpSolver(nodeMgr, src, lib, defaultBuf);
    dpSolver.Solve();
    auto *bestSolution = dpSolver.GetBestSolution();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("Time taken to DP algorithm: %f seconds\n",
           duration.count() / 1000000.0);

    DpSolver::ReportImprovement(net, bestSolution, defaultBuf);
    bestSolution->EmitDOT("result_60.dot");
  }

  {
    NetData net =
        NetData::GenRandomNet(500, 1.0f, 1.2f, 0.001, 0.01); // balanced
    NodeMgr nodeMgr(net.sinks_.size() * 100);
    ClusterSolver solver(nodeMgr, net, defaultBuf);
    BufNode *src = solver.BuildBufferTree();
    src->EmitDOT("balanced.dot");

    auto start = std::chrono::high_resolution_clock::now();
    DpSolver dpSolver(nodeMgr, src, lib, defaultBuf);
    dpSolver.Solve();
    auto *bestSolution = dpSolver.GetBestSolution();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("Time taken to DP algorithm: %f seconds\n",
           duration.count() / 1000000.0);

    DpSolver::ReportImprovement(net, bestSolution, defaultBuf);
    bestSolution->EmitDOT("result_100.dot");
  }
}

int main(int argc, char **argv) { RunExample(); }