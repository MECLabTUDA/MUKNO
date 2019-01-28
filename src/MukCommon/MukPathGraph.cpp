#include "private/muk.pch"

#include "MukPathGraph.h"

#include <numeric>
namespace
{
  using namespace gris;
  using namespace gris::muk;

  void writeNode(std::ostream& os, const MukNode& node)
  {
    os << node.mState << " " << node.children.size() << std::endl;
    for (const auto& node : node.children)
    {
      writeNode(os, node);
    }
  }

  void readNode(std::istream& is, MukNode& node, std::string& line)
  {
    std::istringstream ss(line);
    Vec3d point;
    Vec3d direction;
    size_t numChildren;
    ss >> point;
    ss >> direction;
    ss >> numChildren;
    node.mState = MukState(point, direction);
    node.children.resize(numChildren);
    for (auto& node : node.children)
    {
      std::getline(is, line);
      readNode(is, node, line);
    }
  }
}

namespace gris
{
namespace muk
{
  /**
  */
  size_t MukNode::size() const
  {
    size_t result (1); // self
    for (const auto& node : children)
    {
      result += node.size();
    }
    return result;
  }

  /**
  */
  void MukPathGraph::print(std::ostream& os) const
  {
    os << mSizeStart << std::endl;
    for (const auto& node : mRoots)
    {
      writeNode(os, node);
    }
  }

  /**
  */
  void MukPathGraph::read(std::istream& is) 
  {
    std::string line;
    std::getline(is, line);
    mSizeStart = std::stoi(line);
    while (std::getline(is, line))
    {
      if (!line.empty())
      {
        MukNode node;
        readNode(is, node, line);
        mRoots.push_back(node);
      }
    }
  }

  /**
  */
  size_t MukPathGraph::numNodes() const
  {
    return std::accumulate(mRoots.begin(), mRoots.end(), (size_t)0, [&] (size_t partialSum, const MukNode& node) { return partialSum + node.size(); });
  }
}
}