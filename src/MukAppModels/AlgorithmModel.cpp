#include "private/muk.pch"
#include "AlgorithmModel.h"

#include "MukAlgorithms\AlgorithmFactory.h"
#include "MukAlgorithms\AlgorithmManager.h"
#include "MukAlgorithms\AlgorithmWrapper.h"

#include <gstd/XmlDocument.h>
#include <gstd/XmlNode.h>

#include <utility>

namespace gris
{
namespace muk
{
  /**
  */
  AlgorithmModel::AlgorithmModel()
    : BaseModel()
  {
  }

  /**
  */
  AlgorithmModel::~AlgorithmModel()
  {
  }

  /**
  * \brief prepare all listed algorithms

  here, names of AlgorithmFactory should be used
  */
  std::vector<std::string> AlgorithmModel::listItems()
  {
    auto& fact = GetAlgorithmFactory();
    std::vector<std::string> result;
    fact.getKeys(result);
    return result;
  }

  /**
  */
  void AlgorithmModel::clear()
  {
    mAlgorithmManager.clear();
    mConnections.clear();
  }

  /**
  * \brief add a new algorithm to AlgorithmManager
  */
  unsigned int AlgorithmModel::addAlgorithm(const std::string& name)
  {
    // Add new algorithm instance and return the id of it
    return mAlgorithmManager.addAlgorithm(name);
  }
    
  /**
  * \brief delete algorithm from AlgorithmManager
  */
  void AlgorithmModel::deleteAlgorithm(unsigned int id)
  {
    mAlgorithmManager.removeAlgorithm(id);
  }

  /**
  * \brief connect two algorithms
  */
  void AlgorithmModel::addConnection(unsigned int sourceID, unsigned int sourcePort, unsigned int destinationID, unsigned int destinationPort)
  {
    mAlgorithmManager.connectPorts(sourceID, sourcePort, destinationID, destinationPort);
    auto source      = std::make_pair(sourceID, sourcePort);
    auto destination = std::make_pair(destinationID, destinationPort);
    mConnections.insert(std::make_pair(destination, source));
  }

  /**
  * \brief delete connection between two algorithms
  */
  void AlgorithmModel::deleteConnection(unsigned int sourceID, unsigned int sourcePort, unsigned int destinationID, unsigned int destinationPort)
  {
    mAlgorithmManager.disconnectPorts(sourceID, sourcePort, destinationID, destinationPort);
    auto destination = std::make_pair(destinationID, destinationPort);
    mConnections.erase(destination);
  }

  /**
  * \return algorithm instance by id
  */
  const AlgorithmWrapper& AlgorithmModel::getAlgorithm(unsigned int id) const
  {
    return mAlgorithmManager.getAlgorithm(id);
  }

  /**
  * \return algorithm instance by id
  */
  AlgorithmWrapper& AlgorithmModel::getAlgorithm(unsigned int id)
  {
    return mAlgorithmManager.getAlgorithm(id);
  }

  /** \brief returns the source of a connection

  \param[in] destination an algorithms id and input port
  \return source (algorithm id, port id) of a connected algorithm. If no source is connected, the original input is returned.
  */
  std::pair<unsigned int, unsigned int> AlgorithmModel::getSource(const std::pair<uint, uint>& destination)
  {
    std::pair<uint, uint> result;
    auto iter = mConnections.find(destination);
    if (iter != mConnections.end())
    {
      result = iter->second;
    }
    else
    {
      result = destination;
    }
    return result;
  }

  /** \brief execute all algorithm instances ordert by thier topology
  */
  void AlgorithmModel::update() 
  {
    mAlgorithmManager.update();
  }

  /** \brief saves the algorithm graph into a xml file
  */
  void AlgorithmModel::saveAlgorithm(const std::string& filename) const
  {
    auto doc = XmlDocument::create("Mukno");
    auto root = doc->getRoot().addChild("Algorithm");
    auto ndNodes = root.addChild("AlgNodes");
    auto ndEdges = root.addChild("Edges");
    const auto ids = mAlgorithmManager.getIds();
    for (const auto id : ids)
    {
      auto node = ndNodes.addChild("AlgNode");
      auto& alg = mAlgorithmManager.getAlgorithm(id);
      node.addChild("Name").setValue(alg.name());
      node.addChild("ID").setValue(std::to_string(mAlgorithmManager.getNodeId(id)).c_str());
      auto outIds = mAlgorithmManager.getOutwardConnections(id);
      for (const auto& pair : outIds)
      {
        auto ndEdge = ndEdges.addChild("Edge");
        ndEdge.addChild("SourceId").setValue(std::to_string(id).c_str());
        ndEdge.addChild("SourcePortId").setValue(std::to_string(pair.second.srcPortId).c_str());
        ndEdge.addChild("TargetId").setValue(std::to_string(pair.first).c_str());
        ndEdge.addChild("TargetPortId").setValue(std::to_string(pair.second.dstPortId).c_str());
      }
      auto ndParams = node.addChild("Properties");
      std::vector<std::string> props;
      alg.getPropertyNames(props);
      for (const auto& prop : props)
      {
        std::string value;
        alg.getProperty(prop, value);
        ndParams.addChild(prop.c_str()).setValue(value.c_str());
      }
    }
    XmlDocument::save(filename.c_str(), *doc);
  }

    /** \brief loads an algorithm graph from a xml file

      At start up, where algorithms might be loaded, throwing an exception is not acceptable.
      After that, it is desired.

      \param badPropertiesThrow flag to indicate throwing or logging an expception that occurs during setting properties
    */
    void AlgorithmModel::loadAlgorithm(const std::string& filename, bool badPropertiesThrow)
    {
      try
      {
        auto doc = XmlDocument::read(filename.c_str());
        auto root = doc->getRoot().getChild("Algorithm");
        mAlgorithmManager.clear();
        mConnections.clear();
        auto ndNodes = root.getChild("AlgNodes").getChildren();
        auto ndEdges = root.getChild("Edges").getChildren();
        std::map<unsigned int, unsigned int> lut;
        // load nodes
        for (const auto& node : ndNodes)
        {
          auto algName = node.getChild("Name").getValue();
          auto oldID  = std::atoi(node.getChild("ID").getValue());
          auto tempID = addAlgorithm(algName);
          lut[tempID] = oldID;
          auto& alg   = mAlgorithmManager.getAlgorithm(tempID);
          auto ndParams = node.getChild("Properties").getChildren();
          for (const auto& ndProp : ndParams)
          {
            try
            {
              if (alg.hasProperty(ndProp.getName()))
                alg.setProperty(ndProp.getName(), ndProp.getValue());
            }
            catch(std::exception& e)
            {
              if (badPropertiesThrow)
                throw;
              else
              {
                LOG_LINE << e.what();
              }
            }
          }
        }
        mAlgorithmManager.resetIDs(lut);
        // then load and add edges
        for (const auto& ndEdge : ndEdges)
        {
          auto srcID = std::atoi(ndEdge.getChild("SourceId").getValue());
          auto tarID = std::atoi(ndEdge.getChild("TargetId").getValue());
          auto srcPortID = std::atoi(ndEdge.getChild("SourcePortId").getValue());
          auto tarPortID = std::atoi(ndEdge.getChild("TargetPortId").getValue());
          mAlgorithmManager.connectPorts(srcID, srcPortID, tarID, tarPortID);
          mConnections[std::make_pair(srcID, srcPortID)] = std::make_pair(tarID, tarPortID);
        }
      }
      catch (const MukException&)
      {
        mAlgorithmManager.clear();
        throw;
      }
      catch (const std::exception&)
      {
        mAlgorithmManager.clear();
        throw;
      }
    }

} // namespace muk
} // namespace gris