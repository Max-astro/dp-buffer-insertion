#include "buffering.h"
#include <cstring>
#include <fstream>

#include <ot/taskflow/taskflow.hpp>

void InitTimer(ot::Timer &timer, const char *lpath, const char *vpath) {
  using namespace ot;
  // the first step is to read a library.
  timer.read_celllib(lpath).read_verilog(vpath);

  timer.create_clock("vclk", 1.0);
  timer.update_timing();

  auto clk = timer.clocks().at("vclk");
  float period = clk.period();

  for (auto &pi : timer.primary_inputs()) {
    FOR_EACH_EL_RF(el, rf) {
      timer.set_at(pi.first, el, rf, 0.0);
      timer.set_slew(pi.first, el, rf, 0.0);
    }
  }

  for (auto &po : timer.primary_outputs()) {
    FOR_EACH_EL_RF(el, rf) {
      timer.set_rat(po.first, el, rf, el == MIN ? -1.0 : 0.0);
    }
  }

  timer.update_timing();

  timer.dump_timer(std::cout);
}

bool NeedBuffering(const ot::Point &fr, const ot::Point &to) {
  const int HIGH_FANOUT_THRESHOLD = 10;
  const float DELAY_THRESHOLD = 0.1;

  auto net = to.pin.net();
  if (!net) {
    return false;
  }

  if (net->num_pins() <= HIGH_FANOUT_THRESHOLD) {
    return false;
  }

  float dly = to.at - fr.at;
  // std::cout << point.pin.name() << " dly: " << dly << '\n';
  if (dly > DELAY_THRESHOLD) {
    return true;
  }

  return false;
}

float GetBufDelayOnPath(const ot::Path &path) {
  float bufDly = 0.0;
  std::optional<float> prev_at;
  for (const auto &p : path) {
    if (p.pin.name()[0] == 'B' && p.pin.name()[1] == 'U' &&
        p.pin.name()[2] == 'F') {
      bufDly += p.at - prev_at.value_or(0.0);
    }
    prev_at = p.at;
  }
  return bufDly;
}

bool BufferHighFanoutNets(ot::Timer &timer, const ot::Net *net,
                          const TechLib &lib) {
  NetData netData = NetData::FromTimer(net);
  NodeMgr nodeMgr(netData.sinks_.size() * 100);

  const OTTimingArc &defaultBuf = lib.bufs_[2]; // Use a medium size buffer
  ClusterSolver solver(nodeMgr, netData, defaultBuf);
  BufNode *src = solver.BuildBufferTree();
  // src->EmitDOT("nand45_src.dot");

  DpSolver dpSolver(nodeMgr, src, lib, netData.driverArc_);
  dpSolver.SetDriverTrans(netData.driverTrans_);
  dpSolver.Solve();
  // std::cout << "DP solutions num: " << dpSolver.GetPosSolutions(src).size()
  //           << '\n';

  auto *bestSolution = dpSolver.GetBestSolution();
  if (!dpSolver.IsImproved(netData, bestSolution)) {
    return false;
  }
  dpSolver.ReportImprovement(netData, bestSolution);
  netData.CommitBufferTree(timer, lib, net->name(), bestSolution);
  return true;
}

void DisplayCriticalPathBuffering(ot::Timer &timer, const TechLib &lib) {
  auto paths = timer.report_timing(1, ot::MAX);

  std::vector<std::pair<const ot::Point *, const ot::Point *>> bufferingArcs;
  std::optional<float> prev_at;
  auto frIt = paths[0].begin();
  if (frIt->pin.primary_output()) {
    ++frIt;
  }
  auto toIt = std::next(frIt);
  while (toIt != paths[0].end()) {
    if (NeedBuffering(*frIt, *toIt)) {
      bufferingArcs.emplace_back(&*frIt, &*toIt);
    }
    ++frIt;
    ++toIt;
  }

  // std::cout << "--------------------------------\n";
  // std::cout << "Buffering Drivers: " << bufferingArcs.size() << '\n';
  // for (const auto &p : bufferingArcs) {
  //   std::cout << p.first->pin.name() << " -> " << p.second->pin.name() <<
  //   '\n';
  // }
  // std::cout << "--------------------------------\n\n";

  for (auto &&[fr, to] : bufferingArcs) {
    NetData netData = NetData::FromTimer(fr, to);
    std::cout << "Net Data: " << fr->pin.name() << " -> " << to->pin.name()
              << ", toRAT = " << *to->pin.rat(ot::MAX, ot::RISE) << " "
              << *to->pin.rat(ot::MAX, ot::FALL)
              << ", fanout = " << to->pin.num_fanouts() << '\n';

    // for (auto &sink : netData.sinks_) {
    //   std::cout << "Sink: " << sink.inputCap_ << " " << sink.rat_ << '\n';
    // }
    std::cout << "----------------OPT----------------\n";

    {
      const OTTimingArc &defaultBuf = lib.bufs_[2]; // Use a medium size buffer
      NodeMgr nodeMgr(netData.sinks_.size() * 100);
      ClusterSolver solver(nodeMgr, netData, defaultBuf);
      BufNode *src = solver.BuildBufferTree();
      src->EmitDOT("nand45_src.dot");

      DpSolver dpSolver(nodeMgr, src, lib, netData.driverArc_);
      dpSolver.SetDriverTrans(netData.driverTrans_);
      dpSolver.Solve();
      std::cout << "DP solutions num: " << dpSolver.GetPosSolutions(src).size()
                << '\n';

      // int i = 0;
      // for (auto s : dpSolver.GetPosSolutions(src)) {
      //   std::cout << s->rat_ << " " << s->inCap_ << " " << s->loading_ <<
      //   '\n'; std::string fname = "nand45_result_" + std::to_string(i++) +
      //   ".dot"; s->EmitDOT(fname.c_str());
      // }
      std::cout << "--------------------------------\n";
      auto *bestSolution = dpSolver.GetBestSolution();
      bestSolution->EmitDOT("nand45_best.dot");

      dpSolver.ReportImprovement(netData, bestSolution);

      netData.CommitBufferTree(timer, lib, fr->pin.gate()->name(),
                               bestSolution);
    }

    // -------------- Optimized timing report ----------------
    timer.update_timing();
    // timer.dump_timer(std::cout);
    std::cout << "\n\n-------------- Optimized timing report --------------\n";

    std::cout << "Optimized WNS: " << *timer.report_wns({ot::MAX}) << '\n';
    std::cout << "Optimized Area: " << *timer.report_area() << '\n';

    auto paths = timer.report_timing(1, ot::MAX);
    for (size_t i = 0; i < paths.size(); ++i) {
      std::cout << "----- Critical Path -----\n";
      std::cout << paths[i] << '\n';
    }

    break;
  }
}

void DisplayHFSBuffering(ot::Timer &timer, const TechLib &lib) {
  auto netQue =
      OtAPI::CollectHighFanoutNets(timer, *timer.report_wns({ot::MAX}));
  auto net = netQue.top();
  NetData netData = NetData::FromTimer(net);
  NodeMgr nodeMgr(netData.sinks_.size() * 100);

  const OTTimingArc &defaultBuf = lib.bufs_[2]; // Use a medium size buffer
  ClusterSolver solver(nodeMgr, netData, defaultBuf);
  BufNode *src = solver.BuildBufferTree();
  src->EmitDOT("HFS_src.dot");

  DpSolver dpSolver(nodeMgr, src, lib, netData.driverArc_);
  dpSolver.SetDriverTrans(netData.driverTrans_);
  dpSolver.Solve();
  // std::cout << "DP solutions num: " << dpSolver.GetPosSolutions(src).size()
  //           << '\n';

  auto *bestSolution = dpSolver.GetBestSolution();
  dpSolver.ReportImprovement(netData, bestSolution);
  bestSolution->EmitDOT("HFS_best.dot");

  // dpSolver.ReportImprovement(netData, bestSolution);
  // netData.CommitBufferTree(timer, lib, net->name(), bestSolution);
}

static const char *lpath = nullptr;
static const char *vpath = nullptr;
static const char *output = nullptr;

void ParseArgs(int argc, char **argv) {
  // Parse command line arguments
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-lib") == 0) {
      if (i + 1 < argc) {
        lpath = argv[++i];
      } else {
        std::cerr << "Error: -lib requires a library file path" << std::endl;
        return;
      }
    } else if (strcmp(argv[i], "-o") == 0) {
      if (i + 1 < argc) {
        output = argv[++i];
      } else {
        std::cerr << "Error: -o requires an output file path" << std::endl;
        return;
      }
    } else if (argv[i][0] != '-') {
      // First non-dashed argument is the verilog file
      if (vpath == nullptr) {
        vpath = argv[i];
      } else {
        std::cerr << "Error: Multiple verilog files specified" << std::endl;
        return;
      }
    } else {
      std::cerr << "Error: Unknown option " << argv[i] << std::endl;
      return;
    }
  }
}

int InsertBufferParallel(const std::vector<const ot::Net *> &nets_to_buffer,
                         const TechLib &lib, ot::Timer &timer) {
  tf::Taskflow taskflow;
  tf::Executor executor(16);

  std::mutex mutex;
  int improved = 0;
  for (int i = 0; i < nets_to_buffer.size(); i++) {
    taskflow.emplace([&, idx = i]() {
      auto net = nets_to_buffer[idx];
      auto netData = NetData::FromTimer(net);
      NodeMgr nodeMgr(netData.sinks_.size() * 100);

      const OTTimingArc &defaultBuf = lib.bufs_[2];
      ClusterSolver solver(nodeMgr, netData, defaultBuf);
      BufNode *src = solver.BuildBufferTree();
      // src->EmitDOT("nand45_src.dot");

      DpSolver dpSolver(nodeMgr, src, lib, netData.driverArc_);
      dpSolver.SetDriverTrans(netData.driverTrans_);
      dpSolver.Solve();

      auto *bestSolution = dpSolver.GetBestSolution();
      if (dpSolver.IsImproved(netData, bestSolution)) {
        std::lock_guard<std::mutex> lock(mutex);
        netData.CommitBufferTree(timer, lib, net->name(), bestSolution);
        improved++;
      }
    });
  }

  executor.run(taskflow).wait();
  return improved;
}

void HighFanoutStrategy(ot::Timer &timer, const TechLib &lib,
                        std::unordered_set<std::string> &bufferedNets,
                        int loopLimit) {
  int loop = 0;
  int improved = 1;
  while (loop < loopLimit && improved > 0) {
    float wns = *timer.report_wns({ot::MAX});
    if (wns > 0.0) {
      return;
    }
    auto netQue = OtAPI::CollectHighFanoutNets(timer, wns * 0.7);

    std::vector<const ot::Net *> nets_to_buffer;
    nets_to_buffer.reserve(256);
    // std::cout << "High fanout nets num: " << netQue.size() << '\n';
    if (netQue.empty()) {
      break;
    }

    int cnt = 0;
    while (!netQue.empty() && cnt < 200) {
      auto net = netQue.top();
      netQue.pop();
      if (bufferedNets.count(net->name())) {
        continue;
      }
      bufferedNets.emplace(net->name());

      // std::cout << "Buffering net: " << net->name()
      //           << ", fanout: " << net->num_pins() << ", slack: "
      //           << net->driver()->slack(ot::MAX, ot::RISE).value() << ", "
      //           << net->driver()->slack(ot::MAX, ot::FALL).value()
      //           << ", Delay: " << OtAPI::GetMaxDriverArcDelay(net) << '\n';

      nets_to_buffer.push_back(net);

      cnt++;
    }

    improved = InsertBufferParallel(nets_to_buffer, lib, timer);

    std::cout << "\n\n-------------- Fanout based buffering loop: " << loop
              << " --------------\n";
    timer.update_timing();
    // timer.dump_timer(std::cout);
    std::cout << "Optimized WNS: " << *timer.report_wns({ot::MAX}) << '\n';
    std::cout << "Optimized Area: " << *timer.report_area() << '\n';
    {
      auto paths = timer.report_timing(1, ot::MAX);
      std::cout << "# Critical Path Buf Delay: " << GetBufDelayOnPath(paths[0])
                << '\n';
      // for (size_t i = 0; i < paths.size(); ++i) {
      //   std::cout << "----- Critical Path -----\n";
      //   std::cout << paths[i] << '\n';
      // }
    }

    loop++;
    std::cout << std::endl;
    std::cout << std::flush;
  }
}

void CriticalPathStrategy(ot::Timer &timer, const TechLib &lib,
                          std::unordered_set<std::string> &bufferedNets,
                          int loopLimit) {
  int loop = 0;
  int improved = 1;
  while (loop < loopLimit && improved > 0) {
    auto paths = timer.report_timing(30, ot::MAX);
    std::vector<const ot::Net *> nets_to_buffer;
    nets_to_buffer.reserve(256);
    for (auto &&p : paths) {
      for (auto &&point : p) {
        if (point.pin.is_input() || point.pin.primary_output()) {
          continue;
        }
        auto net = point.pin.net();
        if (!OtAPI::IsHighFanoutNet(*net)) {
          continue;
        }
        if (bufferedNets.count(net->name())) {
          continue;
        }

        bufferedNets.emplace(net->name());
        nets_to_buffer.push_back(net);
      }
    }

    improved = InsertBufferParallel(nets_to_buffer, lib, timer);

    std::cout << "\n\n-------------- Path based buffering loop: " << loop
              << " --------------\n";
    timer.update_timing();
    timer.dump_timer(std::cout);
    std::cout << "Optimized WNS: " << *timer.report_wns({ot::MAX}) << '\n';
    std::cout << "Optimized Area: " << *timer.report_area() << '\n';
    {
      auto paths = timer.report_timing(1, ot::MAX);
      std::cout << "# Critical Path Buf Delay: " << GetBufDelayOnPath(paths[0])
                << '\n';
    }

    loop++;
  }
}

int main(int argc, char **argv) {
  ParseArgs(argc, argv);
  // Check required arguments
  if (lpath == nullptr) {
    std::cerr << "Error: Library file path required (-lib <lpath>)"
              << std::endl;
    return 1;
  }
  if (vpath == nullptr) {
    std::cerr << "Error: Verilog file path required" << std::endl;
    return 1;
  }
  if (output == nullptr) {
    output = "optimized.v"; // Default output file
  }

  ot::Timer timer;
  InitTimer(timer, lpath, vpath);

  // ------------------------------
  std::cout << "Init WNS: " << *timer.report_wns({ot::MAX}) << '\n';
  std::cout << "Area: " << *timer.report_area() << '\n';

  // auto paths = timer.report_timing(1, ot::MAX);
  // for (size_t i = 0; i < paths.size(); ++i) {
  //   std::cout << "----- Critical Path " << i << " -----\n";
  //   std::cout << paths[i] << '\n';
  // }

  Nangate45Lib lib(timer);
  std::unordered_set<std::string> bufferedNets;

  // main opt flow
  for (int i = 0; i < 2; i++) {
    // Strategy 1: buffing critical paths
    CriticalPathStrategy(timer, lib, bufferedNets, 8);

    // // Strategy 2: buffing high fanout nets
    // HighFanoutStrategy(timer, lib, bufferedNets, 5);
  }

  std::cout << "Final WNS: " << *timer.report_wns({ot::MAX}) << '\n';
  std::cout << "Final Area: " << *timer.report_area() << '\n';

  std::cout << "\n\n-------------- Dump to `" << output << "` --------------\n";
  std::ofstream ofs(output);
  timer.dump_verilog(ofs, "top");

  return 0;
}