#include "buffering.h"
#include <functional>
// #include "taskflow.hpp"

BufInvLib::BufInvLib()
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

  std::string driverName = driver_ ? "\n" + driver_->name_ : "";

  os << uid_ << " [label=\"" << uid_ << "\n"
     << GetBufNodeTypeStr(ty_) << driverName << "\nRAT: " << rat_
     << "\nInCap: " << inCap_ << "\nLoading: " << loading
     << "\", shape=ellipse, color=" << GetBufNodeTypeColor(ty_) << "]\n";
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
  for (auto &sink : net_.sinks_) {
    BufNode *node = nodeMgr_.Alloc();
    node->ty_ = BufNodeType::Sink;
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
      node->SetLibCell(&lib[i], inv);
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
    if (!SimilarNodes(*it, node)) {
      solutions.insert(node);
    }
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

void DpSolver::Solve(bool multiThread) {
  BuildDpTree(multiThread);

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

BufNode *DpSolver::GetBestSolution() const {
  BufNode *best = nullptr;
  float bestRat = -std::numeric_limits<float>::max();
  for (auto s : GetPosSolutions(src_)) {
    float srcRat =
        s->rat_ - driverArc_.CalcDelay(DelayType::Rise,
                                       BufInvLib::DEFAULT_TRANS, s->loading_);
    s->rat_ = srcRat;
    if (srcRat > bestRat) {
      best = s;
      bestRat = srcRat;
    }
  }
  return best;
}

void DpSolver::ReportImprovement(const NetData &net, const BufNode *result,
                                 const BufLibCell &driverArc) {
  float oldRat = std::numeric_limits<float>::max();
  float oldLoading = 0.0;
  for (auto &sink : net.sinks_) {
    oldRat = std::min(oldRat, sink.rat_);
    oldLoading += sink.inputCap_;
  }
  float oldDelay = driverArc.CalcDelay(DelayType::Rise,
                                       BufInvLib::DEFAULT_TRANS, oldLoading);
  oldRat -= oldDelay;

  float newDelay = driverArc.CalcDelay(
      DelayType::Rise, BufInvLib::DEFAULT_TRANS, result->loading_);

  printf("Orignal Net's driver pin: RAT = %f, loading = %f, delay = %f;\n"
         "After buffer insertion  : RAT = %f, loading = %f, delay = %f\n\n",
         oldRat, oldLoading, oldDelay, result->rat_, result->loading_,
         newDelay);
}