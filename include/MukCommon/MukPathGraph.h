#pragma once

#include "muk_common_api.h"
#include "MukState.h"

namespace gris
{
namespace muk
{
  /**
  */
  struct MUK_COMMON_API MukNode
  {
    public:
      MukNode() {}
      ~MukNode() {}

      MukNode(const MukNode& o)            = default;
      MukNode& operator=(const MukNode& o) = default;
      MukNode(MukNode&& o)            = default;
      MukNode& operator=(MukNode&& o) = default;
      
    public:
      size_t size() const;

    public:
      MukState mState;
      std::vector<MukNode> children;
  };

  /** \brief Holds a search graph of a path finding algorithm

    Tu support easy visualization of bidrectional RRTs, one can also set a size hint of the first #mSizeStart roots.
  */
  class MUK_COMMON_API MukPathGraph
  {
    public:
      MukPathGraph() : mSizeStart(0) {}
      ~MukPathGraph() = default;
      MukPathGraph(const MukPathGraph& o) = default;
      MukPathGraph& operator=(const MukPathGraph& o) = default;
      MukPathGraph(MukPathGraph&&) = default;
      MukPathGraph& operator=(MukPathGraph&& o) = default;
      
    public:
      void           addRoot(MukNode&& node)  { mRoots.push_back(std::move(node)); }
      const MukNode& getRoot(size_t i) const          { return mRoots[i]; }
      size_t         numRoots()        const  { return mRoots.size(); }
      size_t         numNodes()        const;
      size_t         getSizeStart()    const  { return mSizeStart; };
      void           setSizeStart(size_t n)   { mSizeStart = n; }

      void print(std::ostream& os) const;
      void read(std::istream& is);

    private:
      size_t mSizeStart;
      std::vector<MukNode> mRoots;
  };

}
}