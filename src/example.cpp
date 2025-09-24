#include "buffering.h"

#include <ot/headerdef.hpp>
#include <ot/timer/timer.hpp>

void RunExample(const char *vpath) {
  using namespace ot;
  ot::Timer timer;

  // the first step is to read a library.
  timer.read_celllib("../dataset/NangateOpenCellLibrary_typical.lib")
      .read_verilog(vpath);
  // .read_verilog("../dataset/test_cases/case3.v");
  // .read_sdc("../dataset/st.sdc");

  timer.create_clock("vclk", 1.0);
  timer.update_timing();

  auto clk = timer.clocks().at("vclk");
  float period = clk.period();

  for (auto &pi : timer.primary_inputs()) {
    FOR_EACH_EL_RF(el, rf) {
      timer.set_at(pi.first, el, rf, 0.0);
      timer.set_slew(pi.first, el, rf, 5.0);
    }
  }

  for (auto &po : timer.primary_outputs()) {
    FOR_EACH_EL_RF(el, rf) {
      timer.set_rat(po.first, el, rf, el == MIN ? -1.0 : period);
    }
  }

  timer.update_timing();

  // for (auto &&[name, pi] : timer.primary_inputs()) {
  //   auto v = timer.report_at(name, MAX, RISE);
  //   if (v) {
  //     std::cout << name << ", at = " << *v << "\n";
  //   } else {
  //     std::cout << name << "\n";
  //   }
  // }
  // for (auto &&[name, po] : timer.primary_outputs()) {
  //   auto v = timer.report_at(name, MAX, RISE);
  //   if (v) {
  //     std::cout << name << ", at = " << *v << "\n";
  //   } else {
  //     std::cout << name << "\n";
  //   }
  // }

  timer.dump_timer(std::cout);
  std::cout << "--------------------------------\n";
  // timer.dump_at(std::cout);
  // timer.dump_rat(std::cout);
  // timer.dump_pin_cap(std::cout);
  // timer.dump_slew(std::cout);
  // timer.dump_slack(std::cout);
  // timer.dump_net_load(std::cout);

  // auto sccs = timer.sccs();
  // for (auto &scc : sccs) {
  //   // std::cout << "SCC: size:" << scc._pins.size() << '\n';
  //   for (auto from : scc._pins) {
  //     std::cout << from->name() << "\n";
  //   }
  // }

  // std::cout << "create_clock -period 100 -name virtual_clock\n";
  // for (auto &&[name, pi] : timer.primary_inputs()) {
  //   std::cout << "set_input_delay 0 -min -rise [get_ports " << name << "]\n";
  //   std::cout << "set_input_delay 0 -min -fall [get_ports " << name << "]\n";
  //   std::cout << "set_input_delay 0 -max -rise [get_ports " << name << "]\n";
  //   std::cout << "set_input_delay 0 -max -fall [get_ports " << name << "]\n";
  //   std::cout << "set_input_transition 5 -min -rise [get_ports " << name
  //             << "]\n";
  //   std::cout << "set_input_transition 5 -min -fall [get_ports " << name
  //             << "]\n";
  //   std::cout << "set_input_transition 5 -max -rise [get_ports " << name
  //             << "]\n";
  //   std::cout << "set_input_transition 5 -max -fall [get_ports " << name
  //             << "]\n";
  // }

  // for (auto &&[name, po] : timer.primary_outputs()) {
  //   std::cout << "set_output_delay 0 -min -rise [get_ports " << name << "]"
  //             << " -clock virtual_clock\n";
  //   std::cout << "set_output_delay 0 -min -fall [get_ports " << name << "]"
  //             << " -clock virtual_clock\n";
  //   std::cout << "set_output_delay 0-max -rise [get_ports " << name << "]"
  //             << " -clock virtual_clock\n";
  //   std::cout << "set_output_delay 0 -max -fall [get_ports " << name << "]"
  //             << " -clock virtual_clock\n";
  //   std::cout << "set_load -pin_load 4 [get_ports " << name << "]\n";
  // }

  std::cout << "OT report timing" << '\n';
  auto tns = timer.report_tns({MAX});
  auto wns = timer.report_wns({MAX});
  if (tns) {
    std::cout << "TNS: " << *tns << '\n';
  }
  if (wns) {
    std::cout << "WNS: " << *wns << '\n';
  }

  auto paths = timer.report_timing(5, MAX);

  for (size_t i = 0; i < paths.size(); ++i) {
    std::cout << "----- Critical Path " << i << " -----\n";
    std::cout << paths[i] << '\n';
  }

  // // dump the timing graph to dot format for debugging
  // timer.dump_graph(std::cout);

  // BufInvLib lib;
  // BufLibCell &defaultBuf = lib.bufs_[2]; // Use a medium size buffer
  // {
  //   NetData net =
  //       NetData::GenRandomNet(30, 1.0f, 2.0f, 0.001, 0.01); // unbalanced
  //   NodeMgr nodeMgr(net.sinks_.size() * 100);
  //   ClusterSolver solver(nodeMgr, net, defaultBuf);
  //   BufNode *src = solver.BuildBufferTree();
  //   src->EmitDOT("unbalanced.dot");

  //   auto start = std::chrono::high_resolution_clock::now();
  //   DpSolver dpSolver(nodeMgr, src, lib, defaultBuf);
  //   dpSolver.Solve();
  //   auto *bestSolution = dpSolver.GetBestSolution();
  //   auto end = std::chrono::high_resolution_clock::now();
  //   auto duration =
  //       std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  //   printf("Time taken to DP algorithm: %f seconds\n",
  //          duration.count() / 1000000.0);

  //   DpSolver::ReportImprovement(net, bestSolution, defaultBuf);
  //   bestSolution->EmitDOT("result_30.dot");
  // }

  // {
  //   NetData net =
  //       NetData::GenRandomNet(500, 1.0f, 5.0f, 0.001, 0.01); // unbalanced
  //   NodeMgr nodeMgr(net.sinks_.size() * 100);
  //   ClusterSolver solver(nodeMgr, net, defaultBuf);
  //   BufNode *src = solver.BuildBufferTree();
  //   src->EmitDOT("unbalanced.dot");

  //   auto start = std::chrono::high_resolution_clock::now();
  //   DpSolver dpSolver(nodeMgr, src, lib, defaultBuf);
  //   dpSolver.Solve();
  //   auto *bestSolution = dpSolver.GetBestSolution();
  //   auto end = std::chrono::high_resolution_clock::now();
  //   auto duration =
  //       std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  //   printf("Time taken to DP algorithm: %f seconds\n",
  //          duration.count() / 1000000.0);

  //   DpSolver::ReportImprovement(net, bestSolution, defaultBuf);
  //   bestSolution->EmitDOT("result_60.dot");
  // }

  // {
  //   NetData net =
  //       NetData::GenRandomNet(500, 1.0f, 1.2f, 0.001, 0.01); // balanced
  //   NodeMgr nodeMgr(net.sinks_.size() * 100);
  //   ClusterSolver solver(nodeMgr, net, defaultBuf);
  //   BufNode *src = solver.BuildBufferTree();
  //   src->EmitDOT("balanced.dot");

  //   auto start = std::chrono::high_resolution_clock::now();
  //   DpSolver dpSolver(nodeMgr, src, lib, defaultBuf);
  //   dpSolver.Solve();
  //   auto *bestSolution = dpSolver.GetBestSolution();
  //   auto end = std::chrono::high_resolution_clock::now();
  //   auto duration =
  //       std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  //   printf("Time taken to DP algorithm: %f seconds\n",
  //          duration.count() / 1000000.0);

  //   DpSolver::ReportImprovement(net, bestSolution, defaultBuf);
  //   bestSolution->EmitDOT("result_100.dot");
  // }
}

int main(int argc, char **argv) { RunExample(argv[1]); }