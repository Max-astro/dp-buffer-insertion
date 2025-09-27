#include "buffering.h"
#include <fstream>

void Init(ot::Timer &timer, const char *vpath) {
  using namespace ot;
  // the first step is to read a library.
  timer.read_celllib("../NangateOpenCellLibrary_typical.lib")
      .read_verilog(vpath);

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

void BufferHighFanoutNets(ot::Timer &timer, const ot::Net *net,
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
  netData.CommitBufferTree(timer, lib, net->name(), bestSolution);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <verilog file>" << std::endl;
    return 1;
  }

  char *vpath = argv[1];
  ot::Timer timer;
  Init(timer, vpath);

  // ------------------------------
  std::cout << "Init WNS: " << *timer.report_wns({ot::MAX}) << '\n';
  std::cout << "Area: " << *timer.report_area() << '\n';

  auto paths = timer.report_timing(1, ot::MAX);
  for (size_t i = 0; i < paths.size(); ++i) {
    std::cout << "----- Critical Path " << i << " -----\n";
    std::cout << paths[i] << '\n';
  }

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

  std::cout << "--------------------------------\n";
  std::cout << "Buffering Drivers: " << bufferingArcs.size() << '\n';
  for (const auto &p : bufferingArcs) {
    std::cout << p.first->pin.name() << " -> " << p.second->pin.name() << '\n';
  }
  std::cout << "--------------------------------\n\n";

  Nangate45Lib lib(timer);

  for (auto &&[fr, to] : bufferingArcs) {
    NetData netData = NetData::FromTimer(fr, to);
    std::cout << "Net Data: " << fr->pin.name() << " -> " << to->pin.name()
              << ", toRAT = " << *to->pin.rat(ot::MAX, ot::RISE) << " "
              << *to->pin.rat(ot::MAX, ot::FALL) << '\n';

    // for (auto &sink : netData.sinks_) {
    //   std::cout << "Sink: " << sink.inputCap_ << " " << sink.rat_ << '\n';
    // }
    std::cout << "----------------OPT----------------\n";

    {
      OTTimingArc &defaultBuf = lib.bufs_[2]; // Use a medium size buffer
      NodeMgr nodeMgr(netData.sinks_.size() * 100);
      ClusterSolver solver(nodeMgr, netData, defaultBuf);
      BufNode *src = solver.BuildBufferTree();
      src->EmitDOT("nand45_src.dot");

      DpSolver dpSolver(nodeMgr, src, lib, netData.driverArc_);
      dpSolver.SetDriverTrans(netData.driverTrans_);
      dpSolver.Solve();
      std::cout << "DP solutions num: " << dpSolver.GetPosSolutions(src).size()
                << '\n';

      int i = 0;
      for (auto s : dpSolver.GetPosSolutions(src)) {
        std::cout << s->rat_ << " " << s->inCap_ << " " << s->loading_ << '\n';
        std::string fname = "nand45_result_" + std::to_string(i++) + ".dot";
        s->EmitDOT(fname.c_str());
      }
      std::cout << "--------------------------------\n";
      auto *bestSolution = dpSolver.GetBestSolution();
      bestSolution->EmitDOT("nand45_best.dot");

      netData.CommitBufferTree(timer, lib, fr->pin.gate()->name(),
                               bestSolution);
    }

    // -------------- Optimized timing report ----------------
    timer.update_timing();
    timer.dump_timer(std::cout);
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

  {
    auto netQue = OtAPI::CollectHighFanoutNets(timer);
    std::cout << "High fanout nets num: " << netQue.size() << '\n';
    int cnt = 0;
    while (!netQue.empty() && cnt < 10000) {
      auto net = netQue.top();
      netQue.pop();
      BufferHighFanoutNets(timer, net, lib);
      // std::cout << "Buffered net: " << net->name() << '\n';
      cnt++;
    }
  }

  std::cout << "\n\n-------------- HFS buffered --------------\n";
  timer.update_timing();
  timer.dump_timer(std::cout);
  std::cout << "Optimized WNS: " << *timer.report_wns({ot::MAX}) << '\n';
  std::cout << "Optimized Area: " << *timer.report_area() << '\n';
  {
    auto paths = timer.report_timing(1, ot::MAX);
    for (size_t i = 0; i < paths.size(); ++i) {
      std::cout << "----- Critical Path -----\n";
      std::cout << paths[i] << '\n';
    }
  }
  std::cout << "\n\n-------------- Dump to `optimized.v` --------------\n";
  std::ofstream ofs("optimized.v");
  timer.dump_verilog(ofs, "top");

  return 0;
}