#include "buffering.h"

void BufNode::EmitDOT(std::string filename) const {
  std::ofstream os(filename.c_str(), std::ofstream::out);

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

  os << uid_ << " [label=\"" << GetBufNodeTypeStr(ty_) << "\n"
     << uid_ << "\nRAT: " << rat_ << "\nInCap: " << inCap_
     << "\nLoading: " << loading
     << "\", shape=ellipse, color=" << GetBufNodeTypeColor(ty_) << "]\n";
  for (auto child : children_) {
    os << uid_ << " -> " << child->uid_ << " [style=solid]\n";
  }
  for (auto child : children_) {
    child->EmitDOT(os);
  }
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

void DpSolver::InitDp(const BufNode *node) {
  if (node->ty_ == BufNodeType::Sink) {
    return;
  }

  BufNodeVec2 dp = {std::vector<BufNode *>{}, std::vector<BufNode *>{}};
  dp[0].reserve(DP_SIZE);
  dp[1].reserve(DP_SIZE);
  dp_.emplace(node->uid_, std::move(dp));

  for (auto child : node->children_) {
    InitDp(child);
  }
}

// void ProcessNode(const BufNode *node) {
//   if (node->ty_ == BufNodeType::Sink) {
//     return;
//   }

//   for (auto child : node->children_) {
//     if (child->ty_ == BufNodeType::Sink) {
//       continue;
//     } else if (child->ty_ == BufNodeType::Buffer) {
//       auto& posSolutions = dp_[child->uid_][0];
//       auto& negSolutions = dp_[child->uid_][1];
//     }
//     ProcessNode(child);
//   }
// }

void DpSolver::MaintainFrontier(BufNode *node, BufNodeRbTree &solutions) {
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
    return;
  }

  auto prev = std::prev(it);
  if (CheckDominate(*prev, node)) {
    return;
  }

  while (it != solutions.end()) {
    if (CheckDominate(node, *it)) {
      it = solutions.erase(it);
    } else {
      ++it;
    }
  }

  solutions.insert(node);
}

void DpSolver::MergeChildSolutions(
    std::vector<BufNode *> &srcSolutions,
    const std::vector<BufNode *> &childSolutions) {
  // std::vector<BufNode *> merged;
  // merged.reserve(DP_SIZE * 4);
  BufNodeRbTree merged;

  for (auto srcS : srcSolutions) {
    for (auto childS : childSolutions) {
      auto *dup = nodeMgr_.Dup(srcS);
      dup->AddChild(childS);
      MaintainFrontier(dup, merged);
    }
  }
}

int main() {
  // std::cout << "Hello, World!" << std::endl;
  BufInvLib lib;
  BufLibCell defaultBuf = lib.bufs_[2]; // Use a medium size buffer

  NetData net = NetData::GenRandomNet(30); // unbalanced
  NodeMgr nodeMgr(net.sinks_.size() * 10);
  ClusterSolver solver(nodeMgr, net, defaultBuf);
  BufNode *src = solver.BuildBufferTree();
  src->EmitDOT("unbalanced.dot");

  {
    BufNodeRbTree rbt;

    std::function<void(BufNode *)> dfs = [&](BufNode *src) {
      rbt.insert(src);
      for (auto *node : src->children_) {
        dfs(node);
      };
    };

    dfs(src);

    for (auto *node : rbt) {
      printf("%u %f\n", node->uid_, node->rat_);
    }

    auto n297 = nodeMgr.GetNode(297);
    {
      auto it = rbt.lower_bound(n297);
      assert(it == rbt.begin());
    }

    auto node = nodeMgr.Alloc();
    node->rat_ = 71.0;
    auto it = rbt.lower_bound(node);

    bool m = 1 || ((std::prev(rbt.begin()) == rbt.end()));
    printf("\nit %u %f, m %d\n", (*it)->uid_, (*it)->rat_, m);
  }

  {
    NetData net = NetData::GenRandomNet(30, 1.0f, 1.2f); // balanced
    ClusterSolver solver(nodeMgr, net, defaultBuf);
    BufNode *src = solver.BuildBufferTree();
    src->EmitDOT("balanced.dot");
  }
  return 0;
}