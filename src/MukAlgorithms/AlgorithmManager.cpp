#include "private/muk.pch"
#include "AlgorithmManager.h"
#include "AlgorithmFactory.h"

#include "MukCommon/MukException.h"

#include <boost/graph/topological_sort.hpp>

#include <deque>
#include <chrono>

namespace
{
  using namespace boost;
  using namespace gris::muk;

  using Graph = AlgorithmManager::DirectedGraph;

  struct Vertex
  {
    graph_traits<Graph>::vertex_iterator it;
    graph_traits<Graph>::vertex_iterator end;
  };

  Vertex vertexIterator(const Graph& G)
  {
    Vertex ver;
    graph_traits<Graph>::vertex_iterator vi, vi_end;
    tie(vi, vi_end) = vertices(G);
    ver.it = vi;
    ver.end = vi_end;
    return ver;
  };
}

namespace gris
{
namespace muk
{
  AlgorithmManager::AlgorithmManager()
    : mAlgGraph(0)
  {
  }

  /** \brief creates a new algorithm instance and adds it to std::map<...> mpAlgorithms and boost graph

  \return ID of the new instance
  */
  unsigned int AlgorithmManager::addAlgorithm(const std::string& name)
  {
    auto pObj = GetAlgorithmFactory().create(name);
    auto N = getNextId();
    mpAlgorithms.insert(std::make_pair(N, std::move(pObj)));
    const auto vertex = boost::add_vertex(mAlgGraph);
    mAlgGraph[vertex].id = N;
    return N;
  }

  /** \brief remove algorithm instance from std::map<...> mpAlgorithms and boost graph by given ID
  */
  void AlgorithmManager::removeAlgorithm(unsigned int id)
  {
    auto name = mpAlgorithms[id]->name();
    mpAlgorithms.erase(id);
    {
      auto ver = vertexIterator(mAlgGraph);
      auto verID = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == id; });
      boost::clear_vertex(*verID, mAlgGraph);
      boost::remove_vertex(*verID, mAlgGraph);
    }
    LOG_LINE << "algorithm " << name << " deleted";
  }

  /** \return wrapped insance of the algorithm
  */
  AlgorithmWrapper& AlgorithmManager::getAlgorithm(unsigned int id)
  {
    auto ver = vertexIterator(mAlgGraph);
    auto iter = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == id; });
    if (iter == ver.end)
    {
      throw MUK_EXCEPTION("Algorithm with this ID does not exist:", std::to_string(id).c_str());
    }
    return *mpAlgorithms[id];
  }

  /** \return wrapped insance of the algorithm
  */
  const AlgorithmWrapper& AlgorithmManager::getAlgorithm(unsigned int id) const
  {
    auto ver = vertexIterator(mAlgGraph);
    auto iter = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == id; });
    if (iter == ver.end)
    {
      throw MUK_EXCEPTION("Algorithm with this ID does not exist:", std::to_string(id).c_str());
    }
    return *mpAlgorithms.at(id);
  }

  /** \brief connect to algorithm instances. Set output of source alg. to inout of destination alg.
  */
  void AlgorithmManager::connectPorts(unsigned int srcAlgId, unsigned int srcPortId, unsigned int dstAlgId, unsigned int dstPortId)
  {
    auto ver = vertexIterator(mAlgGraph);
    auto srcID = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == srcAlgId; });
    auto pSrc = mpAlgorithms.find(srcAlgId);
    if (srcID ==ver.end || pSrc==mpAlgorithms.end())
    {
      throw MUK_EXCEPTION("(Source) algorithm with this ID does not exist:", std::to_string(srcAlgId).c_str());
    }
    auto dstID = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == dstAlgId; });
    auto pDst = mpAlgorithms.find(dstAlgId);
    if (dstID==ver.end ||pDst==mpAlgorithms.end())
    {
      throw MUK_EXCEPTION("(Destination) algorithm with this ID does not exist:", std::to_string(dstAlgId).c_str());
    }

    auto typeSrc = pSrc->second->getOutputType(srcPortId);
    auto typeDst = pDst->second->getInputType(dstPortId);
    if (typeSrc != typeDst)
    {
      const auto strSrc = DataTypeNames[typeSrc];
      const auto strDst = DataTypeNames[typeDst];
      const auto msg = (boost::format("Source data type: %s, Target data type: %s") % strSrc % strDst).str();
      throw MUK_EXCEPTION("port type of source and destination mismatch!", msg.c_str());
    }

    pDst->second->setInput(dstPortId, pSrc->second->getOutput(srcPortId));

    boost::add_edge(*srcID, *dstID, mAlgGraph);
    auto edge = boost::edge(*srcID, *dstID, mAlgGraph);
    mAlgGraph[edge.first].srcPortId = srcPortId;
    mAlgGraph[edge.first].dstPortId = dstPortId;

  }

  /** \brief delete connection between to algorithm instances
  */
  void AlgorithmManager::disconnectPorts(unsigned int srcAlgId, unsigned int srcPortId, unsigned int dstAlgId, unsigned int dstPortId)
  {
    auto ver = vertexIterator(mAlgGraph);

    auto srcID = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == srcAlgId; });
    auto pSrc = mpAlgorithms.find(srcAlgId);
    if (srcID == ver.end || pSrc == mpAlgorithms.end())
    {
      throw MUK_EXCEPTION("(Source) algorithm with this ID does not exist:", std::to_string(srcAlgId).c_str());
    }

    auto dstID = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == dstAlgId; });
    auto pDst = mpAlgorithms.find(dstAlgId);
    if (dstID == ver.end || pDst == mpAlgorithms.end())
    {
      throw MUK_EXCEPTION("(Destination) algorithm with this ID does not exist:", std::to_string(dstAlgId).c_str());
    }
    boost::remove_edge(*srcID, *dstID, mAlgGraph);
  }

  /** \brief execute all algorithm instances ordered by their topology
  */
  void AlgorithmManager::update()
  {
    // Perform a topological sort.
    std::deque<int> topo_order;
    boost::topological_sort(mAlgGraph, std::front_inserter(topo_order));
    // update accordingly
    for (auto iter = topo_order.begin(); iter != topo_order.end(); ++iter)
    {
      const auto id = mAlgGraph[*iter].id;
      LOG_LINE << "   " << mpAlgorithms[id]->name() << " (alias: " << mpAlgorithms[id]->getAlias() << ", id " << id << ")...";
      try
      {
        using namespace std::chrono;
        auto pre = high_resolution_clock::now();
        mpAlgorithms[id]->update();
        auto post = high_resolution_clock::now();
        auto s = duration_cast<seconds>(post-pre).count();
        auto ms = duration_cast<milliseconds>(post-pre).count();
        if (s<1)
        {
          LOG_LINE << "     finished after " << ms << "ms";
        }
        else
        {
          LOG_LINE << "     finished after " << s << "s";
        }
      }
      catch (std::exception&)
      {
        LOG_LINE << "exception occured during pipeline processing while updating algorithm with id " << id << "!";
        throw;
      }
    }
  }

  /** \brief returns the ids of all existing algorithms
  */
  std::vector<unsigned int> AlgorithmManager::getIds() const
  {
    std::vector<unsigned int> ids;
    std::for_each(mpAlgorithms.begin(), mpAlgorithms.end(),
      [&] (const auto& pair) 
      {
        ids.push_back(pair.first);
      });
    return ids;
  }

  /** \brief returns the ids of the algorithm a algorithm is connected with

    \param[in] id of the algorithm node
    \return    a list of ids the passed algorithm is connected with
  */
  std::map<unsigned int, AlgEdge> AlgorithmManager::getOutwardConnections(unsigned int id) const
  {
    std::map<unsigned int, AlgEdge> result;
    auto ver   = vertexIterator(mAlgGraph);
    auto srcID = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == id; });
    if (srcID==ver.end)
      return result;
    auto iters = boost::out_edges(*srcID, mAlgGraph);
    std::for_each(iters.first, iters.second,
      [&] (const auto& obj)
      {
        auto target = boost::target(obj, mAlgGraph);
        result[mAlgGraph[target].id].srcPortId = mAlgGraph[obj].srcPortId;
        result[mAlgGraph[target].id].dstPortId = mAlgGraph[obj].dstPortId;
      });
    return result;
  }

  /**
  */
  size_t AlgorithmManager::getNodeId(unsigned int id) const
  {
    auto ver   = vertexIterator(mAlgGraph);
    auto srcID = std::find_if(ver.it, ver.end, [&] (const auto& vertex) { return mAlgGraph[vertex].id == id; });
    if (srcID==ver.end)
      return std::numeric_limits<size_t>::quiet_NaN();
    else
      return mAlgGraph[*srcID].id;
  }

  /** \brief delete whole graph, reset internal id counter
  */
  void AlgorithmManager::clear()
  {
    mAlgGraph.clear();
    mpAlgorithms.clear();
    algoId = 0;
  }

  /** \brief needed only for broken algorithm model loading

    \param lut a lookup table with <currentId, newId>
  */ 
  void AlgorithmManager::resetIDs(const std::map<unsigned int, unsigned int>& lut)
  {
    if (lut.size() != mpAlgorithms.size())
      throw MUK_EXCEPTION_SIMPLE("lookup table size doesn't match with number of algorithms");
    if (std::any_of(mpAlgorithms.begin(), mpAlgorithms.end(),
      [&] (const auto& pair)
      {
        return lut.end() == std::find_if(lut.begin(), lut.end(), [&] (const auto& lutpair)
        {
          return pair.first == lutpair.first;
        });
      }))
    {
      // an existing id wasn't found in the lut
      throw MUK_EXCEPTION_SIMPLE("no match for an ID found in the lookup table");
    }

    std::map<unsigned int, std::shared_ptr<AlgorithmWrapper>> newMap;
    auto ver   = vertexIterator(mAlgGraph);
    std::for_each(ver.it, ver.end, [&] (const auto& vertex)
    {
      unsigned int oldid = mAlgGraph[vertex].id;
      unsigned int newid = lut.at(oldid);
      mAlgGraph[vertex].id = newid;
      newMap.insert(std::make_pair(newid, mpAlgorithms.at(oldid)));
    });
    newMap.swap(mpAlgorithms);
    if (!mpAlgorithms.empty())
    {
      // adjust nextId counter
      auto iter = std::max_element(mpAlgorithms.begin(), mpAlgorithms.end(), [&] (const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; } );
      algoId = iter->first + 1;
    }
  }
}
}