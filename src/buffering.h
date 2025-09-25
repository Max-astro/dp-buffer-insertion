#include <array>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <queue>
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "liberty_helper.rs.h"

#include <ot/timer/timer.hpp>

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
  const ot::Timing &arc_;
  float inCap_;

  float CalcDelay(ot::Tran irf, ot::Tran orf, float trans, float loading) const;
  float CalcBufDelay(ot::Tran irf, float trans, float loading) const;
  float CalcInvDelay(ot::Tran irf, float trans, float loading) const;

  OTTimingArc(const std::string &fr, const std::string &to,
              const ot::Cell *cell);
  const ot::Timing &InitArc(const std::string &fr, const std::string &to,
                            const ot::Cell *cell);
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
    return 0.002; // ns (1e-9 s) }
  }
  const float GetMinCap() const override {
    return 0.5; // fF (1e-15 F) }
  }
  const float GetDefaultTrans() const override { return 0.01; }

  Nangate45Lib(const ot::Timer &timer) : TechLib(timer) { InitLib(); }

  // static const ot::Cellpin *GetCellpin(const ot::Timer &timer,
  //                                      const std::string &cellName);
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

  // static const ot::Cellpin *GetCellpin(const ot::Timer &timer,
  //                                      const std::string &cellName);
};

// TODO: support other liberty files
struct Sky130BufInvLib {
  LibPtr lib_; // For managing liberty's memory allocated from Rust
  std::vector<BufLibCell> bufs_;
  std::vector<BufLibCell> invs_;

  // sky130 lib
  // constexpr static const std::string BUF_NAME = "sky130_fd_sc_hd__buf_";
  // constexpr static const std::string INV_NAME = "sky130_fd_sc_hd__inv_";
  // constexpr static const std::string INPUT_PIN = "A";
  // constexpr static const std::string BUF_OUTPUT_PIN = "X";
  // constexpr static const std::string INV_OUTPUT_PIN = "Y";
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
  // Sky130BufInvLib() : minDelay_(InvB[0]), minCap_(0.001) {
  //   const float SCALE = 1.5;
  //   char name[32];
  //   for (int i = 0; i < 7; i++) {
  //     snprintf(name, sizeof(name), "buf_X%zu", Sizes[i]);
  //     bufs_.push_back(BufLibCell(name, BufK[i] * SCALE, BufB[i],
  //     BufInCap[i]));

  //     snprintf(name, sizeof(name), "inv_X%zu", Sizes[i]);
  //     invs_.push_back(BufLibCell(name, InvK[i] * SCALE, InvB[i],
  //     InvCap[i]));
  //   }
  // }
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
  float inCap_ = 0.0;
  float loading_ = 0.0;
  float rat_ = std::numeric_limits<float>::max();
  const BufLibCell *driver_;
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

  void SetLibCell(const BufLibCell *libCell, bool isInv) {
    driver_ = libCell;
    ty_ = isInv ? BufNodeType::Inverter : BufNodeType::Buffer;
    float dly = libCell->CalcDelay(DelayType::Rise,
                                   Sky130BufInvLib::DEFAULT_TRANS, loading_);
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

  std::vector<BufNode *> TopologicalSort() const;

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

  BufNode *Dup(const BufNode *node) {
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
};

struct NetData {
  // TODO: Need record source pin's timing arc
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
  /// For set comparsion has a stable result, shouldn't compare uid
  // if (std::abs(a->rat_ - b->rat_) < 1e-7) {
  //   return a->uid_ > b->uid_;
  // }
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
    return defaultBuf_.CalcDelay(DelayType::Rise,
                                 Sky130BufInvLib::DEFAULT_TRANS, loading);
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
  const BufLibCell &driverArc_; // Driver's timing arc
  const BufNode *src_;
  const Sky130BufInvLib &libCells_;

  using BufNodeVec2 = std::array<BufNodeVec, 2>;

  // 0: positive, 1: negative
  std::unordered_map<uint32_t, BufNodeVec2> dp_;

  static constexpr size_t DP_SIZE = 16;

  DpSolver(NodeMgr &nodeMgr, const BufNode *src,
           const Sky130BufInvLib &libCells, const BufLibCell &driverArc)
      : nodeMgr_(nodeMgr), src_(src), libCells_(libCells),
        driverArc_(driverArc) {}

  BufNodeVec2 InitDp(const BufNode *node);

  bool SimilarNodes(const BufNode *a, const BufNode *b) const {
    bool ratEq = std::abs(a->rat_ - b->rat_) < libCells_.minDelay_;
    if (a->ty_ == BufNodeType::Init && b->ty_ == BufNodeType::Init) {
      // Compare loading instead of inCap during merge child stage
      bool loadingEq = std::abs(a->loading_ - b->loading_) < libCells_.minCap_;
      return ratEq && loadingEq;
    }
    bool inCapEq = std::abs(a->inCap_ - b->inCap_) < libCells_.minCap_;
    return ratEq && inCapEq;
  }

  // a dom b
  bool CheckDominate(const BufNode *a, const BufNode *b) const {
    if (SimilarNodes(a, b)) {
      return true;
    }

    bool ratGt = a->rat_ > b->rat_;
    if (a->ty_ == BufNodeType::Init && b->ty_ == BufNodeType::Init) {
      // Compare loading instead of inCap during merge child stage
      bool loadingLt = a->loading_ < b->loading_;
      return ratGt && loadingLt;
    }
    bool inCapLt = a->inCap_ < b->inCap_;
    return ratGt && inCapLt;
  }

  const BufNodeVec &GetPosSolutions(const BufNode *node) const {
    return dp_.at(node->uid_)[0];
  }

  const BufNodeVec &GetNegSolutions(const BufNode *node) const {
    return dp_.at(node->uid_)[1];
  }

  // TODO: use taskflow to parallelize the dp tree building
  void BuildDpTree(bool multiThread = false);
  void GenNodeSolutions(const BufNode *node);
  BufNodeVec GenSolutionsByPhase(BufNodeVec &insertBuf, BufNodeVec &insertInv);
  void GenRemoveBufferSolutions(BufNodeVec &candidates, BufNodeRbTree &rbt);
  void InsertLibCell(BufNodeVec &candidates, BufNodeRbTree &rbt, bool inv);
  void MaintainFrontier(BufNode *node, BufNodeRbTree &rbt);

  void MergeChildSolutions(BufNodeVec &srcSolutions,
                           const BufNodeVec &childSolutions);

  void Solve(bool multiThread = false);
  BufNode *GetBestSolution() const;

  static void ReportImprovement(const NetData &net, const BufNode *result,
                                const BufLibCell &driverArc);
};
