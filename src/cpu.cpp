#include "buffering.h"
#include <functional>

// #include "taskflow.hpp"

const ot::Timing *OTTimingArc::InitArc(const std::string &fr,
                                       const std::string &to,
                                       const ot::Cell *cell) {
  auto cellpin = cell->cellpin(to);
  assert(cellpin->direction == ot::CellpinDirection::OUTPUT);
  for (auto &timing : cellpin->timings) {
    if (timing.related_pin == fr) {
      return &timing;
    }
  }
  assert(false && "Timing not found");
  return &cellpin->timings[0];
}

OTTimingArc::OTTimingArc(const std::string &fr, const std::string &to,
                         const ot::Cell *cell)
    : arc_(InitArc(fr, to, cell)), cell_(cell) {
  auto ipin = cell->cellpin(fr);
  assert(ipin->direction == ot::CellpinDirection::INPUT);
  inCap_ = ipin->capacitance.value();
}

OTTimingArc OTTimingArc::ExtractFromPins(const ot::Pin *fr, const ot::Pin *to) {
  auto &frName = fr->cellpin(ot::MAX)->name;
  auto &toName = to->cellpin(ot::MAX)->name;
  auto cell = fr->gate()->cell(ot::MAX);
  return OTTimingArc(frName, toName, cell);
}

float OTTimingArc::CalcDelay(ot::Tran irf, ot::Tran orf, float trans,
                             float loading) const {
  auto dly = arc_->delay(irf, orf, trans, loading);

  assert(dly.has_value());
  return dly.value();
}

float OTTimingArc::CalcAverageDelay(float trans, float loading) const {
  float rise = arc_->cell_rise.value()(trans, loading);
  float fall = arc_->cell_fall.value()(trans, loading);
  return (rise + fall) / 2.0;
}

float OTTimingArc::CalcBufDelay(ot::Tran irf, float trans,
                                float loading) const {
  return CalcDelay(irf, irf, trans, loading);
}

float OTTimingArc::CalcInvDelay(ot::Tran irf, float trans,
                                float loading) const {
  ot::Tran orf = irf == ot::RISE ? ot::FALL : ot::RISE;
  return CalcDelay(irf, orf, trans, loading);
}

const ot::Cell *TechLib::GetCell(const std::string &cellName) {
  auto &celllib = timer_.celllib()[ot::MAX];
  auto &cell = celllib->cells.at(cellName);

  return &cell;
}

void Sky130Lib::InitMockTimer(ot::Timer &timer) {
  using namespace ot;

  // the first step is to read a library.
  timer.read_celllib("../sky130_fd_sc_hd__tt_025C_1v80.lib")
      .read_verilog("../tests/sky130.v");

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
}

void TechLib::InitLib() {
  for (auto sz : GetBufSizes()) {
    std::string cname = GetBufName() + std::to_string(sz);
    auto cell = GetCell(cname);
    auto opin = cell->cellpin(GetBufOutputPin());
    bufs_.emplace_back(OTTimingArc(GetInputPin(), GetBufOutputPin(), cell));
  }

  for (auto sz : GetInvSizes()) {
    std::string cname = GetInvName() + std::to_string(sz);
    auto cell = GetCell(cname);
    auto opin = cell->cellpin(GetInvOutputPin());
    invs_.emplace_back(OTTimingArc(GetInputPin(), GetInvOutputPin(), cell));
  }
}

Sky130BufInvLib::Sky130BufInvLib()
    : lib_(read_lib(std::string("../sky130_fd_sc_hd__tt_025C_1v80.lib"))),
      bufs_(), invs_() {

  for (auto sz : SIZES) {
    std::string name = BUF_NAME + std::to_string(sz);
    auto cell = get_cell(*lib_, std::move(name));
    auto rise = get_timing_table(*cell, std::string(INPUT_PIN),
                                 std::string(BUF_OUTPUT_PIN), true);
    auto fall = get_timing_table(*cell, std::string(INPUT_PIN),
                                 std::string(BUF_OUTPUT_PIN), false);

    float inCap = get_pin_capacitance(*cell, std::string(INPUT_PIN));
    BufLibCell buf(name, TimingArc(INPUT_PIN, std::move(rise), std::move(fall)),
                   inCap);
    // printf("buf: %s, inCap: %f\n", name.c_str(), inCap);
    bufs_.emplace_back(std::move(buf));
  }

  for (auto sz : SIZES) {
    std::string name = INV_NAME + std::to_string(sz);
    auto cell = get_cell(*lib_, std::move(name));
    auto rise = get_timing_table(*cell, std::string(INPUT_PIN),
                                 std::string(INV_OUTPUT_PIN), true);
    auto fall = get_timing_table(*cell, std::string(INPUT_PIN),
                                 std::string(INV_OUTPUT_PIN), false);

    float inCap = get_pin_capacitance(*cell, std::string(INPUT_PIN));
    BufLibCell inv(name, TimingArc(INPUT_PIN, std::move(rise), std::move(fall)),
                   inCap);
    // printf("inv: %s, inCap: %f\n", name.c_str(), inCap);
    invs_.emplace_back(std::move(inv));
  }
}

void BufNode::EmitDOT(const char *filename) const {
  std::ofstream os(filename, std::ofstream::out);

  os << "digraph {\n";
  EmitDOT(os);

  os << "}\n";
  os.close();
}

void BufNode::EmitDOT(std::ofstream &os) const {
  float loading = 0.0;
  for (auto child : children_) {
    loading += child->inCap_;
  }

  std::string driverName = driver_ ? "\n" + driver_->cell_->name : "";

  os << uid_ << " [label=\"" << uid_ << "\n"
     << GetBufNodeTypeStr(ty_) << driverName << "\nRAT: " << rat_;

  if (ty_ != BufNodeType::Src) {
    os << "\nInCap: " << inCap_;
  }
  if (ty_ != BufNodeType::Sink) {
    os << "\nLoading: " << loading;
  }
  os << "\", shape=ellipse, color=" << GetBufNodeTypeColor(ty_) << "]\n";
  for (auto child : children_) {
    os << uid_ << " -> " << child->uid_ << " [style=solid]\n";
  }
  for (auto child : children_) {
    child->EmitDOT(os);
  }
}

bool BufNode::CheckPhase(bool inv) const {
  std::function<bool(const BufNode *)> dfs =
      [&dfs](const BufNode *node) -> bool {
    if (node->ty_ == BufNodeType::Sink) {
      // TODO: sink might have negative phase when we perform rebuffer
      return false;
    }

    bool phase = dfs(node->children_[0]);
    for (int i = 1; i < node->children_.size(); i++) {
      if (phase != dfs(node->children_[i])) {
        node->EmitDOT("phase_error.dot");
        assert(0);
      }
    }

    return node->ty_ == BufNodeType::Inverter ? !phase : phase;
  };

  return dfs(this) == inv;
}

bool BufNode::CheckLoading() const {
  float loading = 0.0;
  for (auto child : children_) {
    loading += child->inCap_;
  }
  bool eq = std::abs(loading - loading_) < 1e-7;
  if (ty_ == BufNodeType::Removed) {
    eq = eq && std::abs(inCap_ - loading_) < 1e-7;
  }
  return eq;
}

void BufNode::ClearRemovedNodes() {
  for (auto *node : TopologicalSort()) {
    std::vector<BufNode *> newChildren;
    for (auto *child : node->children_) {
      if (child->ty_ == BufNodeType::Removed) {
        for (auto *grandChild : child->children_) {
          newChildren.push_back(grandChild);
        }
      } else {
        newChildren.push_back(child);
      }
    }
    node->children_ = std::move(newChildren);
  }
}

std::vector<BufNode *> BufNode::TopologicalSort() const {
  std::vector<BufNode *> sorted;
  std::function<void(BufNode *)> dfs = [&](BufNode *node) {
    for (auto child : node->children_) {
      dfs(child);
    }
    sorted.push_back(node);
  };
  dfs(const_cast<BufNode *>(this));
  return sorted;
}

BufNode *ClusterSolver::InsertBuffer(NodeGroup &group) {
  BufNode *buf = nodeMgr_.Alloc();

  buf->ty_ = BufNodeType::Buffer;
  // buf->rat_ = group.rat_ - GetVirtualBufDelay();
  buf->rat_ = group.rat_ - GetBufDelayByLoading(group.loading_);
  buf->inCap_ = GetVirtualBufCap();
  std::swap(group.sinks_, buf->children_);

  group.Reset();
  return buf;
}

// bool StopCriteron(RatHeap &maxHeap, NodeGroup &group) {}

BufNode *ClusterSolver::BuildBufferTree() {
  float rat = net_.srcRAT_;
  float loading = net_.inCap_;

  // use a heap to store the sinks
  RatHeap maxHeap(NodeRatLT);
  uint32_t sinkIdx = 0;
  for (auto &sink : net_.sinks_) {
    BufNode *node = nodeMgr_.AllocSink(sinkIdx++);
    node->rat_ = sink.rat_;
    node->inCap_ = sink.inputCap_;
    maxHeap.push(node);
  }

  // buffer popped out sinks
  const size_t FANOUT_LIMIT = 8;
  NodeGroup group;
  while (!maxHeap.empty()) {
    BufNode *node = maxHeap.top();
    maxHeap.pop();
    group.AddNode(node);

    if (NeedInsertBuffer(group)) {
      BufNode *buf = InsertBuffer(group);
      maxHeap.push(buf);
    }
  }

  assert(!group.sinks_.empty());
  BufNode *src = nodeMgr_.Alloc();
  src->ty_ = BufNodeType::Src;
  src->rat_ = group.rat_;
  std::swap(src->children_, group.sinks_);

  return src;
}

DpSolver::BufNodeVec2 DpSolver::InitDp(const BufNode *node) {
  // if (node->ty_ == BufNodeType::Sink) {
  //   return;
  // }

  BufNodeVec2 dp = {std::vector<BufNode *>{}, std::vector<BufNode *>{}};
  dp[0].reserve(DP_SIZE);
  dp[1].reserve(DP_SIZE);
  // dp_.emplace(node->uid_, std::move(dp));

  return dp;
}

void DpSolver::GenNodeSolutions(const BufNode *node) {
  if (node->ty_ == BufNodeType::Sink) {
    return;
  }

  // `Candidates` means buffer/inverter libcell not yet determined
  // `Solutions` means this node is a valid solution
  auto solutions = InitDp(node);
  auto &posCandidates = solutions[0];
  auto &negCandidates = solutions[1];

  // Sink nodes will be skipped during the merge
  // so we need to add them to the candidates
  bool hasSink = false;
  posCandidates.push_back(nodeMgr_.Alloc());
  for (auto child : node->children_) {
    if (child->ty_ == BufNodeType::Sink) {
      posCandidates[0]->AddChild(child);
      hasSink = true;
    }
  }

  // If any of the child nodes is a sink
  // then we cannot construct negative solution
  // TODO: sinks might have negative phase when we perform rebuffer
  if (!hasSink) {
    negCandidates.push_back(nodeMgr_.Alloc());
  }

  for (auto child : node->children_) {
    if (child->ty_ != BufNodeType::Buffer) {
      continue;
    }

#if ENABLE_ASSERT
    { // Debug: check phase
      for (auto s : GetPosSolutions(child)) {
        assert(s->CheckPhase(false));
      }
      for (auto s : GetNegSolutions(child)) {
        assert(s->CheckPhase(true));
      }
    }
#endif

    MergeChildSolutions(posCandidates, GetPosSolutions(child));
    MergeChildSolutions(negCandidates, GetNegSolutions(child));
  }

#if ENABLE_ASSERT
  {
    // Debug: check fanout size
    for (auto s : posCandidates) {
      assert(s->children_.size() == node->children_.size());
      assert(s->loading_ > 0.0);
    }
    for (auto s : negCandidates) {
      assert(s->children_.size() == node->children_.size());
      assert(s->loading_ > 0.0);
    }
  }
#endif

  auto posDup = nodeMgr_.Dup(posCandidates);
  auto negDup = nodeMgr_.Dup(negCandidates);
  // To generate positive solutions, we need to insert buffer to positive
  // candidates and insert inverter to negative candidates;
  // Or, we can remove buffer from positive candidates
  solutions[0] = GenSolutionsByPhase(posCandidates, negCandidates);
  // For negative solutions, we need to insert buffer to negative candidates
  // and insert inverter to positive candidates.
  // Or, we can remove buffer from negative candidates
  solutions[1] = GenSolutionsByPhase(negDup, posDup);

#if ENABLE_ASSERT
  {
    // Debug: check phase
    for (auto s : solutions[0]) {
      if (!s->CheckPhase(false)) {
        s->EmitDOT("pos_phase_error.dot");
        assert(0);
      }
    }
    for (auto s : solutions[1]) {
      if (!s->CheckPhase(true)) {
        s->EmitDOT("neg_phase_error.dot");
        assert(0);
      }
    }
  }
#endif

  // std::lock_guard<std::mutex> lock(*mutex_);
  dp_.emplace(node->uid_, std::move(solutions));
}

// Nodes in both pos and neg vectors will be modified
BufNodeVec DpSolver::GenSolutionsByPhase(BufNodeVec &insertBuf,
                                         BufNodeVec &insertInv) {
  // All new solutions will be stored in the rbt
  // in order to keep the Pareto frontier easier.
  BufNodeRbTree rbt;

#if ENABLE_ASSERT
  {
    for (auto s : insertBuf) {
      assert(s->ty_ == BufNodeType::Init);
    }
    for (auto s : insertInv) {
      assert(s->ty_ == BufNodeType::Init);
    }
  }
#endif

  GenRemoveBufferSolutions(insertBuf, rbt);

  InsertLibCell(insertBuf, rbt, false);
  InsertLibCell(insertInv, rbt, true);

  BufNodeVec ret;
  ret.reserve(rbt.size());
  std::copy(rbt.begin(), rbt.end(), std::back_inserter(ret));

  return ret;
}

void DpSolver::GenRemoveBufferSolutions(BufNodeVec &candidates,
                                        BufNodeRbTree &rbt) {
  for (auto *s : candidates) {
    auto *dup = nodeMgr_.Dup(s);
    dup->RemoveBuffer();
    assert(dup->inCap_ > 0);
    MaintainFrontier(dup, rbt);
  }
}

void DpSolver::InsertLibCell(BufNodeVec &candidates, BufNodeRbTree &rbt,
                             bool inv) {
  for (auto s : candidates) {
    auto const &lib = inv ? libCells_.invs_ : libCells_.bufs_;
    for (int i = 0; i < lib.size(); i++) {
      // Reuse the original node during the final iteration.
      BufNode *node = (i == lib.size() - 1) ? s : nodeMgr_.Dup(s);
      node->SetLibCell(&lib[i], inv, libCells_.GetDefaultTrans());
      // assert(node->CheckPhase(inv));
      MaintainFrontier(node, rbt);
    }
  }
}

void DpSolver::MaintainFrontier(BufNode *node, BufNodeRbTree &solutions) {
#if ENABLE_ASSERT
  assert(node);
  assert(node->rat_ < std::numeric_limits<float>::max());
  assert(node->ty_ == BufNodeType::Init || node->ty_ == BufNodeType::Src ||
         node->inCap_ > 0);
  assert(node->ty_ != BufNodeType::Sink || node->loading_ > 0);
#endif

  if (solutions.empty()) {
    solutions.insert(node);
    return;
  }

  auto it = solutions.lower_bound(node);
  if (it == solutions.begin()) {
    // node has the largest RAT
    // if (!SimilarNodes(*it, node)) {
    //   bool res;
    //   std::tie(it, res) = solutions.insert(node);
    //   assert(res);
    // }
  } else {
    // check if the node is dominated by the previous (larger RAT) node
    it = std::prev(it);
    if (CheckDominate(*it, node)) {
      return;
    }
  }

  // The rest of elements's RAT < node's RAT,
  // prune elements are dominated by current node
  while (it != solutions.end()) {
    if (CheckDominate(node, *it)) {
      nodeMgr_.Recycle(*it);
      it = solutions.erase(it);
    } else {
      ++it;
    }
  }

  solutions.insert(node);
}

void DpSolver::MergeChildSolutions(BufNodeVec &srcSolutions,
                                   const BufNodeVec &childSolutions) {
  if (srcSolutions.empty()) {
    return;
  }

  BufNodeRbTree merged;
  for (auto srcS : srcSolutions) {
    for (auto childS : childSolutions) {
      auto *dup = nodeMgr_.Dup(srcS);
      dup->AddChild(childS);
      MaintainFrontier(dup, merged);
    }
  }

  srcSolutions.clear();
  srcSolutions.reserve(merged.size());
  std::copy(merged.begin(), merged.end(), std::back_inserter(srcSolutions));
}

void DpSolver::BuildDpTree(bool multiThread) {
  // if (multiThread) {
  //   tf::Executor executor;
  //   tf::Taskflow taskflow;

  //   std::unordered_map<BufNode *, tf::Task> tasks;
  //   for (auto *node : src_->TopologicalSort()) {
  //     if (node->ty_ == BufNodeType::Src || node->ty_ == BufNodeType::Sink) {
  //       continue;
  //     }
  //     auto task = taskflow.emplace([this, node]() { GenNodeSolutions(node);
  //     }); tasks[node] = task; for (auto child : node->children_) {
  //       if (child->ty_ != BufNodeType::Sink) {
  //         task.succeed(tasks.at(child));
  //       }
  //     }
  //   }

  //   executor.run(taskflow).wait();
  // return;
  // }
  for (auto *node : src_->TopologicalSort()) {
    if (node->ty_ == BufNodeType::Src) {
      continue;
    }
    GenNodeSolutions(node);
  }
}

void DpSolver::BuildSrcSolutions() {
  // Source node equals to net's driver pin
  // we only need positive solutions
  auto solutions = InitDp(src_);
  auto *root = nodeMgr_.Alloc();
  root->ty_ = BufNodeType::Src;
  for (auto child : src_->children_) {
    if (child->ty_ == BufNodeType::Sink) {
      root->AddChild(child);
    }
  }
  solutions[0].push_back(root);

  for (auto *child : src_->children_) {
    if (child->ty_ == BufNodeType::Sink) {
      continue;
    }
    MergeChildSolutions(solutions[0], GetPosSolutions(child));
  }
  dp_.emplace(src_->uid_, std::move(solutions));
}

void DpSolver::Solve(bool multiThread) {
  BuildDpTree(multiThread);
  BuildSrcSolutions();
}

BufNode *DpSolver::GetBestSolution() const {
  BufNode *best = nullptr;
  float trans = driverTrans_.value_or(libCells_.GetDefaultTrans());
  float bestRat = -std::numeric_limits<float>::max();
  for (auto s : GetPosSolutions(src_)) {
    float srcRat = s->rat_ - driverArc_.CalcAverageDelay(trans, s->loading_);
    s->rat_ = srcRat;
    if (srcRat > bestRat) {
      best = s;
      bestRat = srcRat;
    }
  }
  return best;
}

void DpSolver::ReportImprovement(const NetData &net, const BufNode *result) {
  float oldRat = std::numeric_limits<float>::max();
  float oldLoading = 0.0;
  for (auto &sink : net.sinks_) {
    oldRat = std::min(oldRat, sink.rat_);
    oldLoading += sink.inputCap_;
  }
  float oldDelay = driverArc_.CalcAverageDelay(
      driverTrans_.value_or(libCells_.GetDefaultTrans()), oldLoading);
  oldRat -= oldDelay;

  float newDelay = driverArc_.CalcAverageDelay(libCells_.GetDefaultTrans(),
                                               result->loading_);

  printf("Orignal Net's driver pin: RAT = %f, loading = %f, delay = %f;\n"
         "After buffer insertion  : RAT = %f, loading = %f, delay = %f\n\n",
         oldRat, oldLoading, oldDelay, result->rat_, result->loading_,
         newDelay);
}

SinkNode SinkNode::FromOTPin(const ot::Pin *pin) {
  auto rrat = pin->rat(ot::MAX, ot::RISE);
  auto frat = pin->rat(ot::MAX, ot::FALL);
  float rat = 0.0;
  if (rrat.has_value() && frat.has_value()) {
    rat = (rrat.value() + frat.value()) / 2.0;
  } else if (rrat.has_value()) {
    rat = rrat.value();
  } else if (frat.has_value()) {
    rat = frat.value();
  } else {
    assert(false && "Rat not found");
  }

  float inCap =
      (pin->cap(ot::MAX, ot::RISE) + pin->cap(ot::MAX, ot::FALL)) / 2.0;
  return SinkNode(inCap, rat);
}

NetData NetData::CreateNetData(const ot::Pin *frPin, const ot::Pin *toPin,
                               ot::Tran tran) {
  NetData netData;
  netData.driverArc_ = OTTimingArc::ExtractFromPins(frPin, toPin);
  netData.driverPin_ = toPin;
  netData.driverTrans_ = tran;

  auto *net = toPin->net();
  netData.sinks_.reserve(net->num_pins() - 1);
  uint32_t sinkIdx = 0;
  for (auto *pin : net->pins()) {
    if (pin == toPin) {
      continue;
    }

    netData.sinks_.emplace_back(SinkNode::FromOTPin(pin));
    netData.srcRAT_ = std::min(netData.srcRAT_, netData.sinks_.back().rat_);
    netData.inCap_ += netData.sinks_.back().inputCap_;

    netData.sinkMap_.emplace(sinkIdx++, pin);
  }

  return netData;
}

NetData NetData::FromTimer(const ot::Point *cellIn, const ot::Point *cellOut) {
  auto *frPin = &cellIn->pin;
  auto *toPin = &cellOut->pin;
  auto *net = toPin->net();

  return CreateNetData(frPin, toPin, cellIn->transition);
}

NetData NetData::FromTimer(const ot::Net *net) {
  auto *toPin = net->driver();
  float rslack = toPin->slack(ot::MAX, ot::RISE).value();
  float fslack = toPin->slack(ot::MAX, ot::FALL).value();
  float critSlack = std::min(rslack, fslack);
  ot::Tran critTran = ot::RISE;

  std::cout << "toPin: " << toPin->name() << " rise slack: " << rslack << ", fall slack: " << fslack
            << '\n';

  // Find the critical fanin
  const ot::Pin *critFr = nullptr;
  for (auto *fanin : toPin->fanins()) {
    assert(fanin->is_cell_arc());
    auto &fr = fanin->from();
    auto rslack = fr.slack(ot::MAX, ot::RISE).value();
    auto fslack = fr.slack(ot::MAX, ot::FALL).value();
    std::cout << "fr pin: " << fr.name() << "  rise slack: " << rslack
              << ", fall slack: " << fslack << '\n';
    if (std::abs(critSlack - rslack) < 1e-5) {
      critFr = &fr;
      critTran = ot::RISE;
      break;
    } else if (std::abs(critSlack - fslack) < 1e-5) {
      critFr = &fr;
      critTran = ot::FALL;
      break;
    }
  }
  assert(critFr && "Critical fanin not found");

  return CreateNetData(critFr, toPin, critTran);
}

void NetData::CommitBufferTree(ot::Timer &timer, const TechLib &techLib,
                               const std::string &driverName,
                               BufNode *solution) {
  std::unordered_map<const BufNode *, std::string> nodePinMap;
  auto topoOrd = solution->TopologicalSort();

  int removedCount = 0;
  for (auto *node : topoOrd) {
    if (node->ty_ == BufNodeType::Removed) {
      ++removedCount;
    }
  }

  solution->ClearRemovedNodes();

  auto tmp = solution->TopologicalSort();
  printf("Total node: %zu, removedCount = %d, remainCount = %zu\n",
         topoOrd.size(), removedCount, tmp.size());

  // preprocess: remove the removed nodes

  uint32_t gateIdx = 0;
  for (auto *node : topoOrd) {
    ++gateIdx;

    if (node->ty_ == BufNodeType::Sink) {
      // disconnect the sink's input pin from the net
      auto *sink = sinkMap_.at(node->sinkIdx_);
      timer.disconnect_pin(sink->name());
      nodePinMap.emplace(node, sink->name());
      continue;
    }

    if (node->ty_ == BufNodeType::Src) {
      for (auto *child : node->children_) {
        timer.connect_pin(nodePinMap.at(child), driverPin_->net()->name());
      }
      continue;
    }

    if (node->ty_ == BufNodeType::Removed) {
      continue;
    }

    assert(node->driver_ && "node should be a buffer or inverter");

    // Five steps to insert a buffer:
    // 1. insert a buffer/inverter gate
    // 2. insert a net
    // 3. add new inserted gate's input pin to nodePinMap,
    //    waiting to be connected to its driver's output pin
    // 4. connect new inserted gate's output pin to the net
    // 5. connect children's input pin to the net

    auto &libCellName = node->driver_->cell_->name;
    std::string gateName = driverName + "_" + std::to_string(gateIdx);
    std::string netName = driverName + "_net_" + std::to_string(gateIdx);

    auto oPinName = node->ty_ == BufNodeType::Buffer
                        ? techLib.GetBufOutputPin()
                        : techLib.GetInvOutputPin();
    std::string cellOutputPinName = gateName + ":" + oPinName;
    std::string cellInputPinName = gateName + ":" + techLib.GetInputPin();

    // step 1 and 2
    timer.insert_gate(gateName, libCellName).insert_net(netName);

    // step 3
    nodePinMap.emplace(node, std::move(cellInputPinName));

    // step 4
    timer.connect_pin(cellOutputPinName, netName);

    // step 5
    for (auto *child : node->children_) {
      timer.connect_pin(nodePinMap.at(child), netName);
    }
  }
}

float OtAPI::GetMaxDriverArcDelay(const ot::Net *net) {
  using namespace ot;
  float maxDelay = 0.0;
  for (auto *fanin : net->driver()->fanins()) {
    FOR_EACH_RF_RF(irf, orf) {
      auto delay = fanin->delay(MAX, irf, orf);
      if (delay.has_value()) {
        maxDelay = std::max(maxDelay, delay.value());
      }
    }
  }

  return maxDelay;
}

NetQueue OtAPI::CollectHighFanoutNets(const ot::Timer &timer) {
  constexpr int HIGH_FANOUT_THRESHOLD = 20;
  constexpr float DELAY_THRESHOLD = 0.1;

  NetQueue netQueue;
  for (auto &&[name, net] : timer.nets()) {
    if (net.driver() && net.driver()->primary_input()) {
      continue;
    }
    if (net.num_pins() < HIGH_FANOUT_THRESHOLD) {
      continue;
    }
    if (GetMaxDriverArcDelay(&net) < DELAY_THRESHOLD) {
      continue;
    }
    netQueue.push(&net);
  }
  return netQueue;
}