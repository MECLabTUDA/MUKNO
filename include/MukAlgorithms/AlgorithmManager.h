#pragma once
#include "AlgorithmWrapper.h"
#include "muk_algorithms_api.h"

#include <boost/graph/adjacency_list.hpp>
namespace gris
{
  namespace muk
  {
    /** \brief Boost Graph Ids are ... complicated
    */
    struct AlgNode
    {
      size_t id;
    };

    /** \brief saves the portIds
    */
    struct AlgEdge
    {
      unsigned int srcPortId;
      unsigned int dstPortId;
    };

    /**
    * Class holds all algorithms and algorithm actions
    */
    class MUK_ALGO_API AlgorithmManager
    {
      public:
        using DirectedGraph = boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, AlgNode, AlgEdge>;

      public:
        AlgorithmManager();

      public:
        unsigned int            addAlgorithm(const std::string& name);
        void                    removeAlgorithm(unsigned int id);
        AlgorithmWrapper&       getAlgorithm(unsigned int id);
        const AlgorithmWrapper& getAlgorithm(unsigned int id) const;
        void                    connectPorts(unsigned int srcAlgId, unsigned int srcPortId, unsigned int dstAlgId, unsigned int dstPortId);
        void                    disconnectPorts(unsigned int srcAlgId, unsigned int srcPortId, unsigned int dstAlgId, unsigned int dstPortId);

      public:
        size_t                    getNumberOfAlgorithms() const { return mpAlgorithms.size(); }
        std::vector<unsigned int> getIds() const;
        std::map<unsigned int, AlgEdge> getOutwardConnections(unsigned int id) const;
        size_t                    getNodeId(unsigned int) const;
        void                      resetIDs(const std::map<unsigned int, unsigned int>& lut);

      public:
        void update();
        void clear();

      private:
        unsigned int getNextId() { return algoId++; };

      private:
        std::map<unsigned int, std::shared_ptr<AlgorithmWrapper>> mpAlgorithms;
        DirectedGraph mAlgGraph;
        unsigned int algoId = 0;
    };
  }
}