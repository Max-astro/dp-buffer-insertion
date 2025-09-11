#include <cassert>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <vector>

struct BufLibCell {
  std::string name_;
  float k_;
  float b_;
  float inCap_;

  BufLibCell(std::string name, float k, float b, float inCap)
      : name_(std::move(name)), k_(k), b_(b), inCap_(inCap) {}

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
constexpr float BufInCap[] = {0.002103, 0.001727, 0.002400, 0.004620,
                              0.007007, 0.009187, 0.013639};
constexpr float InvK[] = {4.474317, 2.352907, 1.345238, 0.956021,
                          0.730050, 0.534276, 0.440415};
constexpr float InvB[] = {0.016583, 0.015086, 0.016207, 0.016781,
                          0.017008, 0.018485, 0.020665};
constexpr float InvCap[] = {0.002302, 0.004459, 0.009004, 0.013272,
                            0.017653, 0.026011, 0.033442};

struct BufInvLib {
  std::vector<BufLibCell> bufs_;
  std::vector<BufLibCell> invs_;

  const float minDelay_; // For measuring minimum delay interval
  const float minCap_;

  BufInvLib() : minDelay_(InvB[0]), minCap_(0.001) {
    char name[32];
    for (int i = 0; i < 7; i++) {
      snprintf(name, sizeof(name), "buf_X%zu", Sizes[i]);
      bufs_.push_back(BufLibCell(name, BufK[i], BufB[i], BufInCap[i]));

      snprintf(name, sizeof(name), "inv_X%zu", Sizes[i]);
      invs_.push_back(BufLibCell(name, InvK[i], InvB[i], InvCap[i]));
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

inline const char *GetBufNodeTypeStr(BufNodeType ty) {
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
  const uint32_t uid_;
  float inCap_ = 0.0;
  float loading_ = 0.0;
  float rat_ = std::numeric_limits<float>::max();
  BufLibCell *driver_;
  std::vector<BufNode *> children_;

  BufNode(uint32_t uid) : uid_(uid), driver_(nullptr) {}
  BufNode(const BufNode &node) = delete;
  BufNode &operator=(const BufNode &node) = delete;

  void Reset() {
    ty_ = BufNodeType::Init;
    inCap_ = 0;
    loading_ = 0;
    rat_ = std::numeric_limits<float>::max();
    driver_ = nullptr;
    children_.clear();
  }

  void AddChild(BufNode *child) {
    rat_ = std::min(rat_, child->rat_);
    loading_ += child->inCap_;
    children_.push_back(child);
  }

  void SetLibCell(BufLibCell *libCell) {
    driver_ = libCell;
    rat_ -= libCell->CalcDelay(loading_);
    inCap_ = libCell->inCap_;
  }

  void EmitDOT(std::string filename = "buffer_tree.dot") const;
  void EmitDOT(std::ofstream &os) const;
};

inline const char *GetBufNodeTypeColor(BufNodeType ty) {
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

  BufNode *Dup(const BufNode *node) {
    BufNode *dup = Alloc();
    dup->ty_ = node->ty_;
    dup->inCap_ = node->inCap_;
    dup->rat_ = node->rat_;
    dup->driver_ = node->driver_;
    dup->children_ = node->children_;
    return dup;
  }

  //   void Recycle(BufNode *node) {
  //     node->Reset();
  //     freeNodes_.push_back(node);
  //   }

  // For debug only
  BufNode *GetNode(uint32_t uid) {
    for (auto node : usedNodes_) {
      if (node->uid_ == uid) {
        return node;
      }
    }
    return nullptr;
  }
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

inline auto NodeRatLT = [](BufNode *a, BufNode *b) {
  if (std::abs(a->rat_ - b->rat_) < 1e-7) {
    return a->uid_ < b->uid_;
  }
  return a->rat_ < b->rat_;
};

inline auto NodeRatGT = [](BufNode *a, BufNode *b) {
  if (std::abs(a->rat_ - b->rat_) < 1e-7) {
    return a->uid_ > b->uid_;
  }
  return a->rat_ > b->rat_;
};

using RatHeap =
    std::priority_queue<BufNode *, std::vector<BufNode *>, decltype(NodeRatLT)>;

using BufNodeRbTree = std::set<BufNode *, decltype(NodeRatGT)>;

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

  BufNode *InsertBuffer(NodeGroup &group);
  BufNode *BuildBufferTree();

  // bool StopCriteron(RatHeap &maxHeap, NodeGroup &group) {}
};

struct DpSolver {
  NodeMgr &nodeMgr_;
  const BufNode *src_;
  const BufInvLib &libCells_;

  using BufNodeVec = std::vector<BufNode *>;
  using BufNodeVec2 = std::array<BufNodeVec, 2>;

  // 0: positive, 1: negative
  std::unordered_map<uint32_t, BufNodeVec2> dp_;

  static constexpr size_t DP_SIZE = 16;

  DpSolver(NodeMgr &nodeMgr, const BufNode *src, const BufInvLib &libCells)
      : nodeMgr_(nodeMgr), src_(src), libCells_(libCells) {
    InitDp(src);
  }

  void InitDp(const BufNode *node);

  bool SimilarNodes(const BufNode *a, const BufNode *b) {
    bool ratEq = std::abs(a->rat_ - b->rat_) < libCells_.minDelay_;
    bool inCapEq = std::abs(a->inCap_ - b->inCap_) < libCells_.minCap_;
    return ratEq && inCapEq;
  }

  // a dom b
  bool CheckDominate(const BufNode *a, const BufNode *b) {
    if (SimilarNodes(a, b)) {
      return true;
    }

    bool ratGt = a->rat_ > b->rat_;
    bool inCapLt = a->inCap_ < b->inCap_;
    return ratGt && inCapLt;
  }

  // void ProcessNode(const BufNode *node);
  void MaintainFrontier(BufNode *node, BufNodeRbTree &solutions);

  void MergeChildSolutions(std::vector<BufNode *> &srcSolutions,
                           const std::vector<BufNode *> &childSolutions);
};
