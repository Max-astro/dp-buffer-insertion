#include <cassert>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

struct BufLibCell {
  std::string name_;
  float k_;
  float b_;

  BufLibCell(std::string name, float k, float b)
      : name_(std::move(name)), k_(k), b_(b) {}

  static constexpr float DEFAULT_TRANS = 0.01;

  float CalcDelay(float loading) const {
    // TODO: read cell's delay LUT to calculate NLDM delay
    return k_ * loading + b_;
  }
};

constexpr size_t Sizes[] = {1, 2, 4, 6, 8, 12, 16};
constexpr float BufK[] = {6.012892, 2.484284, 1.429871, 1.022770,
                          0.833302, 0.621765, 0.577835};
constexpr float BufB[] = {0.055511, 0.086697, 0.094935, 0.078714,
                          0.080080, 0.083737, 0.087594};
constexpr float InvK[] = {4.474317, 2.352907, 1.345238, 0.956021,
                          0.730050, 0.534276, 0.440415};
constexpr float InvB[] = {0.016583, 0.015086, 0.016207, 0.016781,
                          0.017008, 0.018485, 0.020665};

struct BufInvLib {
  std::vector<BufLibCell> bufs_;
  std::vector<BufLibCell> invs_;

  BufInvLib() {
    char name[32];
    for (int i = 0; i < 7; i++) {
      snprintf(name, sizeof(name), "buf_X%zu", Sizes[i]);
      bufs_.push_back(BufLibCell(name, BufK[i], BufB[i]));

      snprintf(name, sizeof(name), "inv_X%zu", Sizes[i]);
      invs_.push_back(BufLibCell(name, InvK[i], InvB[i]));
    }
  }
};

enum class BufNodeType {
  Init,
  Buffer,
  Inverter,
  Removed,
  Sink,
  Src,
};

const char *GetBufNodeTypeStr(BufNodeType ty) {
  switch (ty) {
  case BufNodeType::Init:
    return "Init";
  case BufNodeType::Buffer:
    return "Buffer";
  case BufNodeType::Inverter:
    return "Inverter";
  case BufNodeType::Removed:
    return "Removed";
  case BufNodeType::Sink:
    return "Sink";
  case BufNodeType::Src:
    return "Src";
  }
}

struct BufNode {
  // bool phase_ = true; // true: non-inverting, false: inverting
  // bool isSink_ = false;
  BufNodeType ty_;
  uint32_t uid_;
  float inCap_ = 0.0;
  float rat_ = 0.0;
  BufLibCell *driver_;
  std::vector<BufNode *> children_;

  BufNode(uint32_t uid) : uid_(uid), driver_(nullptr) {}

  void Reset() {
    ty_ = BufNodeType::Init;
    inCap_ = 0;
    rat_ = 0;
    driver_ = nullptr;
    children_.clear();
  }

  void EmitDOT(std::string filename = "buffer_tree.dot") const;
  void EmitDOT(std::ofstream &os) const;
};

void BufNode::EmitDOT(std::string filename) const {
  std::ofstream os(filename.c_str(), std::ofstream::out);

  os << "digraph {\n";
  EmitDOT(os);

  os << "}\n";
  os.close();
}

const char *GetBufNodeTypeColor(BufNodeType ty) {
  switch (ty) {
  case BufNodeType::Init:
    return "red";
  case BufNodeType::Buffer:
    return "blue";
  case BufNodeType::Inverter:
    return "green";
  case BufNodeType::Removed:
    return "gray";
  case BufNodeType::Sink:
    return "sienna";
  case BufNodeType::Src:
    return "orange";
  }
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

struct NodeMgr {
  size_t total_;
  size_t allocCount_ = 0;
  std::vector<BufNode *> freeNodes_;
  std::vector<BufNode *> usedNodes_;

  NodeMgr(size_t total) : total_(total) {
    freeNodes_.reserve(total_);
    for (size_t i = 0; i < total_; i++) {
      freeNodes_.push_back(new BufNode(allocCount_++));
    }
  }

  ~NodeMgr() {
    for (auto node : freeNodes_) {
      delete node;
    }
    for (auto node : usedNodes_) {
      delete node;
    }
  }

  void Realloc(size_t total) {
    total_ = total;
    freeNodes_.reserve(total_);
    for (size_t i = 0; i < total_; i++) {
      freeNodes_.push_back(new BufNode(allocCount_++));
    }
  }

  BufNode *Alloc() {
    if (freeNodes_.empty()) {
      Realloc(total_ * 2);
    }
    BufNode *node = freeNodes_.back();
    freeNodes_.pop_back();
    usedNodes_.push_back(node);
    return node;
  }

  //   void Recycle(BufNode *node) {
  //     node->Reset();
  //     freeNodes_.push_back(node);
  //   }
};

struct SinkNode {
  float inputCap_;
  float rat_;

  SinkNode(float inputCap, float rat) : inputCap_(inputCap), rat_(rat) {}
};

struct NetData {
  float srcRAT_ = std::numeric_limits<float>::max();
  float inCap_ = 0.0;
  std::vector<SinkNode> sinks_;

  static NetData GenRandomNet(size_t numSinks, float ratMin = 1.0f,
                              float ratMax = 100.0f, float loadMin = 0.0001f,
                              float loadMax = 0.1f) {
    NetData net;
    net.sinks_.reserve(numSinks);

    // std::default_random_engine engine(std::random_device{}());
    std::default_random_engine engine(42); // Fixed seed for reproducibility
    std::uniform_real_distribution<float> ratDist(ratMin, ratMax);
    std::uniform_real_distribution<float> loadDist(loadMin, loadMax);

    for (size_t i = 0; i < numSinks; i++) {
      net.sinks_.push_back(SinkNode(loadDist(engine), ratDist(engine)));
      net.srcRAT_ = std::min(net.srcRAT_, net.sinks_.back().rat_);
      net.inCap_ += net.sinks_.back().inputCap_;
    }
    return net;
  }
};

auto ratCompare = [](BufNode *a, BufNode *b) {
  if (std::abs(a->rat_ - b->rat_) < 1e-7) {
    return a->uid_ < b->uid_;
  }
  return a->rat_ < b->rat_;
};
using RatHeap = std::priority_queue<BufNode *, std::vector<BufNode *>,
                                    decltype(ratCompare)>;

struct ClusterSolver {
private:
  // Helper data structure
  struct NodeGroup {
    float rat_ = std::numeric_limits<float>::max();
    float loading_;
    std::vector<BufNode *> sinks_;

    void Reset() {
      rat_ = std::numeric_limits<float>::max();
      loading_ = 0;
      sinks_.clear();
    }

    void AddNode(BufNode *sink) {
      rat_ = std::min(rat_, sink->rat_);
      loading_ += sink->inCap_;
      sinks_.push_back(sink);
    }
  };

public:
  NodeMgr &nodeMgr_;
  const NetData &net_;
  const BufLibCell &defaultBuf_;

  ClusterSolver(NodeMgr &nodeMgr, const NetData &net,
                const BufLibCell &defaultBuf)
      : nodeMgr_(nodeMgr), net_(net), defaultBuf_(defaultBuf) {}

  // TODO: use typical buffer's input capacitance
  float GetVirtualBufCap() const { return 0.02; }

  float GetVirtualBufDelay() const { return 0.05; }

  float GetBufDelayByLoading(float loading) const {
    // TODO: use typical buffer's input capacitance
    return defaultBuf_.CalcDelay(loading);
  }

  // TODO: use more practical values
  int GetFanoutThreshold() const { return 4; }
  float GetLoadingThreshold() const { return GetVirtualBufCap() * 4; }

  bool NeedInsertBuffer(const NodeGroup &bufferdSinks) {
    return bufferdSinks.loading_ > GetLoadingThreshold() ||
           bufferdSinks.sinks_.size() >= GetFanoutThreshold();
  }

  BufNode *InsertBuffer(NodeGroup &group) {
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

  BufNode *BuildBufferTree() {
    float rat = net_.srcRAT_;
    float loading = net_.inCap_;

    // use a heap to store the sinks
    RatHeap maxHeap(ratCompare);
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
};

struct DpSolver {
  NodeMgr &nodeMgr_;
  const BufNode *src_;
  const BufInvLib &libCells_;

  DpSolver(NodeMgr &nodeMgr, const BufNode *src, const BufInvLib &libCells)
      : nodeMgr_(nodeMgr), src_(src), libCells_(libCells) {}
};

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
    NetData net = NetData::GenRandomNet(30, 1.0f, 1.2f); // balanced
    ClusterSolver solver(nodeMgr, net, defaultBuf);
    BufNode *src = solver.BuildBufferTree();
    src->EmitDOT("balanced.dot");
  }
  return 0;
}