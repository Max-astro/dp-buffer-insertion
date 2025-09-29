#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <queue>
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "liberty_helper.rs.h"

// #include <ot/timer/timer.hpp>

#pragma push_macro("DEBUG")
#ifdef DEBUG
#undef DEBUG
#endif
#include <ot/timer/timer.hpp>
#pragma pop_macro("DEBUG")

using LibPtr = rust::Box<LibDb>;
using TimingTbl2DPtr = rust::Box<TimingTbl2D>;

enum class DelayType { Rise, Fall };

// Consider single output cell only
struct TimingArc {
  std::string ipin_;
  std::array<TimingTbl2DPtr, 2> tbl_; // 0: rise, 1: fall

  TimingArc(std::string ipin, TimingTbl2DPtr riseTbl, TimingTbl2DPtr fallTbl)
      : ipin_(std::move(ipin)), tbl_({std::move(riseTbl), std::move(fallTbl)}) {
  }

  double CalcDelay(DelayType ty, double trans, double loading) const {
    if (ty == DelayType::Rise) {
      return lookup(*tbl_[0], trans, loading);
    } else {
      return lookup(*tbl_[1], trans, loading);
    }
  }
};

// Libcells only has one timing arc
struct BufLibCell {
  static constexpr float DEFAULT_TRANS = 0.01;

  std::string name_;
  TimingArc arcs_;
  float inCap_;

  BufLibCell(std::string name, TimingArc arc, float inCap)
      : name_(std::move(name)), arcs_(std::move(arc)), inCap_(inCap) {}

  float CalcDelay(DelayType ty, double trans, double loading) const {
    return arcs_.CalcDelay(ty, trans, loading);
  }

  // const TimingArc &GetArc(const char *ipin) const {
  //   for (auto &arc : arcs_) {
  //     if (arc.ipin_ == ipin) {
  //       return arc;
  //     }
  //   }
  //   assert(false && "Timing arc not found");
  // }
};

struct OTTimingArc {
  const ot::Cell *cell_;
  const ot::Timing *arc_;
  float inCap_;

  float CalcDelay(ot::Tran irf, ot::Tran orf, float trans, float loading) const;
  float CalcAverageDelay(float trans, float loading) const;
  float CalcBufDelay(ot::Tran irf, float trans, float loading) const;
  float CalcInvDelay(ot::Tran irf, float trans, float loading) const;

  OTTimingArc() : cell_(nullptr), arc_(nullptr), inCap_(0.0) {}

  OTTimingArc(const std::string &fr, const std::string &to,
              const ot::Cell *cell);
  const ot::Timing *InitArc(const std::string &fr, const std::string &to,
                            const ot::Cell *cell);

  static OTTimingArc ExtractFromPins(const ot::Pin *fr, const ot::Pin *to);
};

struct TechLib {
  virtual const char *GetBufName() const = 0;
  virtual const char *GetInvName() const = 0;
  virtual const char *GetInputPin() const = 0;
  virtual const char *GetBufOutputPin() const = 0;
  virtual const char *GetInvOutputPin() const = 0;

  virtual const std::vector<int> &GetBufSizes() const = 0;
  virtual const std::vector<int> &GetInvSizes() const = 0;

  virtual const float GetMinDelay() const = 0;
  virtual const float GetMinCap() const = 0;

  virtual const float GetDefaultTrans() const = 0;

  TechLib(const ot::Timer &timer) : timer_(timer) {}

  void InitLib();

  const ot::Cell *GetCell(const std::string &cellName);

  const ot::Timer &timer_;

  std::vector<OTTimingArc> bufs_;
  std::vector<OTTimingArc> invs_;
};

struct Nangate45Lib : public TechLib {
  const char *GetBufName() const override { return "BUF_X"; }
  const char *GetInvName() const override { return "INV_X"; }
  const char *GetInputPin() const override { return "A"; }
  const char *GetBufOutputPin() const override { return "Z"; }
  const char *GetInvOutputPin() const override { return "ZN"; }

  const std::vector<int> bufInvSize_ = {1, 2, 4, 8, 16, 32};
  const std::vector<int> &GetBufSizes() const override { return bufInvSize_; }
  const std::vector<int> &GetInvSizes() const override { return bufInvSize_; }

  // For measuring minimum delay interval
  const float GetMinDelay() const override {
    return 0.005; // ns (1e-9 s) }
  }
  const float GetMinCap() const override {
    return 0.1; // fF (1e-15 F) }
  }
  const float GetDefaultTrans() const override { return 0.02; }

  Nangate45Lib(const ot::Timer &timer) : TechLib(timer) { InitLib(); }
};

struct Sky130Lib : public TechLib {
  const char *GetBufName() const override { return "sky130_fd_sc_hd__buf_"; }
  const char *GetInvName() const override { return "sky130_fd_sc_hd__inv_"; }
  const char *GetInputPin() const override { return "A"; }
  const char *GetBufOutputPin() const override { return "X"; }
  const char *GetInvOutputPin() const override { return "Y"; }

  const std::vector<int> bufInvSize_ = {1, 2, 4, 6, 8, 12, 16};
  const std::vector<int> &GetBufSizes() const override { return bufInvSize_; }
  const std::vector<int> &GetInvSizes() const override { return bufInvSize_; }

  // For measuring minimum delay interval
  const float GetMinDelay() const override { return 0.005; }
  const float GetMinCap() const override { return 0.001; }
  const float GetDefaultTrans() const override { return 0.01; }

  Sky130Lib(const ot::Timer &timer) : TechLib(timer) { InitLib(); }

  // For testing
  static void InitMockTimer(ot::Timer &timer);
};

// TODO: support other liberty files
struct Sky130BufInvLib {
  LibPtr lib_; // For managing liberty's memory allocated from Rust
  std::vector<BufLibCell> bufs_;
  std::vector<BufLibCell> invs_;

  // sky130 lib
  static constexpr const char *BUF_NAME = "sky130_fd_sc_hd__buf_";
  static constexpr const char *INV_NAME = "sky130_fd_sc_hd__inv_";
  static constexpr const char *INPUT_PIN = "A";
  static constexpr const char *BUF_OUTPUT_PIN = "X";
  static constexpr const char *INV_OUTPUT_PIN = "Y";

  constexpr static const int SIZES[] = {1, 2, 4, 6, 8, 12, 16};

  // For measuring minimum delay interval
  static constexpr float minDelay_ = 0.005;
  static constexpr float minCap_ = 0.001;

  static constexpr float DEFAULT_TRANS = 0.01;

  Sky130BufInvLib();
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
    return "Source";
  }
}

struct BufNode {
  // bool phase_ = true; // true: non-inverting, false: inverting
  // bool isSink_ = false;
  BufNodeType ty_;
  const uint32_t uid_;
  const uint32_t sinkIdx_ = std::numeric_limits<uint32_t>::max();
  float inCap_ = 0.0;
  float loading_ = 0.0;
  float rat_ = std::numeric_limits<float>::max();
  const OTTimingArc *driver_;
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

  void SetLibCell(const OTTimingArc *libCell, bool isInv, float trans) {
    driver_ = libCell;
    ty_ = isInv ? BufNodeType::Inverter : BufNodeType::Buffer;
    float dly = libCell->CalcAverageDelay(trans, loading_);
    assert(dly > 0);
    rat_ -= dly;

    inCap_ = libCell->inCap_;
  }

  void RemoveBuffer() {
    ty_ = BufNodeType::Removed;
    // If buf/inv is removed, it means current node's fanouts will be driven
    // directly by its parent, so its input capacitance is equal to its
    // loading
    inCap_ = loading_;
  }

  uint32_t CountBufDepth() const;
  std::vector<BufNode *> TopologicalSort() const;

  void ClearRemovedNodes();

  // Debug utils
  bool CheckPhase(bool inv) const;
  bool CheckLoading() const;

  void EmitDOT(const char *filename = "buffer_tree.dot") const;
  void EmitDOT(std::ofstream &os) const;
};

using BufNodeVec = std::vector<BufNode *>;

inline const char *GetBufNodeTypeColor(BufNodeType ty) {
  switch (ty) {
  case BufNodeType::Init:
    return "darkorchid1";
  case BufNodeType::Buffer:
    return "deepskyblue";
  case BufNodeType::Inverter:
    return "lime";
  case BufNodeType::Removed:
    return "gold";
  case BufNodeType::Sink:
    return "gray";
  case BufNodeType::Src:
    return "red";
  }
}

struct NodeMgr {
  size_t total_;
  size_t allocCount_ = 0;
  std::vector<BufNode *> freeNodes_;
  std::unordered_set<BufNode *> usedNodes_;

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
    // printf("NodeMgr::Realloc: %zu -> %zu\n", total_, total);

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
    usedNodes_.insert(node);
    return node;
  }

  BufNode *AllocSink(uint32_t sinkIdx) {
    BufNode *node = Alloc();
    node->ty_ = BufNodeType::Sink;
    uint32_t *p = const_cast<uint32_t *>(&node->sinkIdx_);
    *p = sinkIdx;
    return node;
  }

  BufNode *Dup(const BufNode *node) {
    assert(node->sinkIdx_ == std::numeric_limits<uint32_t>::max() &&
           "Sink node should not be copied");

    BufNode *dup = Alloc();
    dup->ty_ = node->ty_;
    dup->inCap_ = node->inCap_;
    dup->loading_ = node->loading_;
    dup->rat_ = node->rat_;
    dup->driver_ = node->driver_;
    dup->children_ = node->children_;
    return dup;
  }

  std::vector<BufNode *> Dup(const std::vector<BufNode *> &nodes) {
    std::vector<BufNode *> dupNodes;
    dupNodes.reserve(nodes.size());
    for (auto node : nodes) {
      dupNodes.push_back(Dup(node));
    }
    return dupNodes;
  }

  void Recycle(BufNode *node) {
    node->Reset();
    usedNodes_.erase(node);
    freeNodes_.push_back(node);
  }

  void Reset() {
    while (!usedNodes_.empty()) {
      Recycle(*usedNodes_.begin());
    }
  }

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

  static SinkNode FromOTPin(const ot::Pin *pin);
};

struct NetData {
  // TODO: Need record source pin's timing arc
  OTTimingArc driverArc_;
  const ot::Pin *driverPin_ = nullptr;
  float driverTrans_ = 0.0;
  float srcRAT_ = std::numeric_limits<float>::max();
  float inCap_ = 0.0;
  std::vector<SinkNode> sinks_;
  std::unordered_map<uint32_t, const ot::Pin *> sinkMap_;

  NetData() : driverArc_() {}

  void CommitBufferTree(ot::Timer &timer, const TechLib &techLib,
                        const std::string &driverName, BufNode *solution);

  static NetData FromTimer(const ot::Point *cellIn, const ot::Point *cellOut);
  static NetData FromTimer(const ot::Net *otNet);
  static NetData CreateNetData(const ot::Pin *frPin, const ot::Pin *toPin,
                               ot::Tran tran);

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

inline auto ClusterNodeCmp = [](BufNode *a, BufNode *b) {
  // if (a->CountBufDepth() != b->CountBufDepth()) {
  //   return a->CountBufDepth() > b->CountBufDepth();
  // }

  float va = a->rat_, vb = b->rat_;
  va -= std::pow(2, a->CountBufDepth()) / 2.0;
  vb -= std::pow(2, b->CountBufDepth()) / 2.0;
  if (std::abs(va - vb) < 1e-7) {
    return a->uid_ < b->uid_;
  }
  return va < vb;
};

struct ParetoFrontierCmp {
  ParetoFrontierCmp(float dlyEps, float capEps)
      : dlyEps_(dlyEps), capEps_(capEps) {}

  bool operator()(const BufNode *a, const BufNode *b) const {
    bool ratEq = std::abs(a->rat_ - b->rat_) < dlyEps_;
    if (!ratEq) {
      return a->rat_ > b->rat_;
    }

    bool capEq, capLt;
    if (a->ty_ == BufNodeType::Init) {
      capEq = std::abs(a->loading_ - b->loading_) < capEps_;
      capLt = a->loading_ < b->loading_;
    } else {
      capEq = std::abs(a->inCap_ - b->inCap_) < capEps_;
      capLt = a->inCap_ < b->inCap_;
    }

    return capEq || capLt;
  }

  float dlyEps_;
  float capEps_;
};

using RatHeap = std::priority_queue<BufNode *, std::vector<BufNode *>,
                                    decltype(ClusterNodeCmp)>;

using BufNodeRbTree = std::set<BufNode *, ParetoFrontierCmp>;

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
  const OTTimingArc &defaultBuf_;

  ClusterSolver(NodeMgr &nodeMgr, const NetData &net,
                const OTTimingArc &defaultBuf)
      : nodeMgr_(nodeMgr), net_(net), defaultBuf_(defaultBuf) {}

  // TODO: use typical buffer's input capacitance
  float GetVirtualBufCap() const { return defaultBuf_.inCap_; }
  float GetDefaultTrans() const { return 0.02; }

  float GetVirtualBufDelay() const {
    // Use FO4 delay
    return defaultBuf_.CalcAverageDelay(GetDefaultTrans(),
                                        4 * GetVirtualBufCap());
  }

  float GetBufDelayByLoading(float loading) const {
    // TODO: use typical buffer's input capacitance
    const float scale = 20.0;
    return defaultBuf_.CalcAverageDelay(GetDefaultTrans(), loading * scale);
  }

  // TODO: use more practical values
  int GetFanoutThreshold() const { return 5; }
  float GetLoadingThreshold() const { return GetVirtualBufCap() * 6; }

  bool NeedInsertBuffer(const NodeGroup &bufferdSinks) {
    return bufferdSinks.loading_ > GetLoadingThreshold() ||
           bufferdSinks.sinks_.size() >= GetFanoutThreshold();
  }

  BufNode *InsertBuffer(NodeGroup &group);
  BufNode *BuildBufferTree();

  // byGroup:
  // true : drive bare sinks by one buffer
  // false: each bare sinks will be driven by one buffer
  void SinkWiseBuffer(BufNode *src, bool byGroup = false);

  // bool StopCriteron(RatHeap &maxHeap, NodeGroup &group) {}
};

struct DpSolver {
  NodeMgr &nodeMgr_;
  const OTTimingArc &driverArc_; // Driver's timing arc
  const BufNode *src_;
  const TechLib &libCells_;

  std::optional<float> driverTrans_ = {};
  void SetDriverTrans(float trans) { driverTrans_ = trans; }

  using BufNodeVec2 = std::array<BufNodeVec, 2>;

  // 0: positive, 1: negative
  std::unordered_map<uint32_t, BufNodeVec2> dp_;

  static constexpr size_t DP_SIZE = 16;

  DpSolver(NodeMgr &nodeMgr, const BufNode *src, const TechLib &libCells,
           const OTTimingArc &driverArc)
      : nodeMgr_(nodeMgr), src_(src), libCells_(libCells),
        driverArc_(driverArc) {}

  BufNodeVec2 InitDp(const BufNode *node);

  bool SimilarNodes(const BufNode *a, const BufNode *b) const {
    bool ratEq = std::abs(a->rat_ - b->rat_) < libCells_.GetMinDelay();
    if (a->ty_ == BufNodeType::Init && b->ty_ == BufNodeType::Init) {
      // Compare loading instead of inCap during merge child stage
      bool loadingEq =
          std::abs(a->loading_ - b->loading_) < libCells_.GetMinCap();
      return ratEq && loadingEq;
    }
    bool inCapEq = std::abs(a->inCap_ - b->inCap_) < libCells_.GetMinCap();
    return ratEq && inCapEq;
  }

  // a dom b
  bool CheckDominate(const BufNode *a, const BufNode *b) const {
    if (SimilarNodes(a, b)) {
      return true;
    }

    bool ratEq = std::abs(a->rat_ - b->rat_) < libCells_.GetMinDelay();
    bool ratGE = a->rat_ > b->rat_ || ratEq;
    if (a->ty_ == BufNodeType::Init && b->ty_ == BufNodeType::Init) {
      // Compare loading instead of inCap during merge child stage
      bool loadingEq =
          std::abs(a->loading_ - b->loading_) < libCells_.GetMinCap();
      bool loadingLE = a->loading_ < b->loading_ || loadingEq;
      return ratGE && loadingLE;
    }
    bool inCapEq = std::abs(a->inCap_ - b->inCap_) < libCells_.GetMinCap();
    bool inCapLE = a->inCap_ < b->inCap_ || inCapEq;
    return ratGE && inCapLE;
  }

  const BufNodeVec &GetPosSolutions(const BufNode *node) const {
    return dp_.at(node->uid_)[0];
  }

  const BufNodeVec &GetNegSolutions(const BufNode *node) const {
    return dp_.at(node->uid_)[1];
  }

  BufNodeRbTree CreateParetoFrontierKeeper() const {
    return BufNodeRbTree(
        ParetoFrontierCmp(libCells_.GetMinDelay(), libCells_.GetMinCap()));
  }

  // TODO: use taskflow to parallelize the dp tree building
  void BuildDpTree(bool multiThread = false);
  void BuildSrcSolutions();
  void GenNodeSolutions(const BufNode *node);
  BufNodeVec GenSolutionsByPhase(BufNodeVec &insertBuf, BufNodeVec &insertInv);
  void GenRemoveBufferSolutions(BufNodeVec &candidates, BufNodeRbTree &rbt);
  void InsertLibCell(BufNodeVec &candidates, BufNodeRbTree &rbt, bool inv);
  void MaintainFrontier(BufNode *node, BufNodeRbTree &rbt);

  void MergeChildSolutions(BufNodeVec &srcSolutions,
                           const BufNodeVec &childSolutions);

  void Solve(bool multiThread = false);
  BufNode *GetBestSolution() const;

  bool IsImproved(const NetData &net, const BufNode *result) const;

  void ReportImprovement(const NetData &net, const BufNode *result);
};

struct NetComparator {
  bool operator()(const ot::Net *a, const ot::Net *b) const {
    return a->num_pins() < b->num_pins();
  }
};

using NetQueue =
    std::priority_queue<const ot::Net *, std::vector<const ot::Net *>,
                        NetComparator>;

struct OtAPI {
  static float GetMaxDriverArcDelay(const ot::Net *net);
  static bool IsHighFanoutNet(const ot::Net &net);
  static NetQueue CollectHighFanoutNets(const ot::Timer &timer);
};