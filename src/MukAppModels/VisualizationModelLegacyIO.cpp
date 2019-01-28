#include "private/muk.pch"
#include "private/VisualizationModelLegacyIO.h"

#include "VisualizationModel.h"

#include "MukCommon/MukException.h"
#include "MukCommon/mukIO.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/version.h"
#include "MukCommon/MukStringToolkit.h"

#include "MukVisualization/CameraConfiguration.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisualObject.h"
#include "MukVisualization/VisObstacle.h"
#include "MukVisualization/VisPathCollection.h"

#include "gstd/XmlNode.h"

#include <boost/filesystem.hpp>

#include <memory>

using namespace gris;
using namespace gris::muk;

namespace
{
  namespace fs = boost::filesystem;
  // searches for node <version> ... </version> and returns the LibVersion (converted from the corresponding string) from it
  gris::muk::LibVersion getLibVersion(const std::vector<gris::XmlNode>& xmlNodes);
  Vec3d fromString(const char* string);
  
  template<class T>
  struct LoadNode
  {
    static void load(XmlNode& node, std::shared_ptr<T> pObj);
  };
}

namespace gris
{
namespace muk
{
  /**
  */
  bool VisualizationModelLegacyIO::load(const XmlNode& root, MukScene* pScene, VisScene* dummyVisScene, VisualizationModel* pModel)
  {
    auto children = root.getChildren();
    // version 
    auto libVersion = getLibVersion(children);

    if (libVersion == LibVersion(0, 0, 0, 0))
      return load_v_0_0_0_0(root, pScene, dummyVisScene, pModel);
    if (libVersion < LibVersion(0, 2, 0, 0))
      throw MUK_EXCEPTION_SIMPLE("Unable to load scene-file. \nThis variant of saving a scene is no longer supported.");
    if (libVersion < LibVersion(0, 3, 0, 0))
      return load_v_0_2_0_0(root, pScene, dummyVisScene, pModel);
    if (libVersion < LibVersion(0, 4, 0, 0))
      return load_v_0_3_0_0(root, pScene, dummyVisScene, pModel);
    return false;
  }

  /**
    apparently no legacy file with Mukpaths / PathCollections exists for this version, so only reading of paths necessary (atm)
  */
  bool VisualizationModelLegacyIO::load_v_0_0_0_0(const XmlNode& root, MukScene* pScene, VisScene* dummyVisScene, VisualizationModel* pModel)
  {
    auto node = root.getChild("Obstacles");
    {
      auto ndObstacles = node.getChildren();
      for (auto& ndObj : ndObstacles)
      {
        auto tmp = ndObj.getChild("Name");
        const std::string key = tmp.getValue();
        auto pObj = dummyVisScene->getObstacle(key);
        LoadNode<VisualObject>::load(ndObj, pObj);
      }
    }
    // vtkWindow
    node = root.getChild("Visualization");
    {
      CameraConfiguration config;
      auto subNode = node.getChild("Camera");
      auto subsubNode = subNode.getChild("FocalPoint");
      config.setFocalPoint(fromString(subsubNode.getValue()));
      subsubNode = subNode.getChild("Position");
      config.setPosition(fromString(subsubNode.getValue()));
      subsubNode = subNode.getChild("ViewUp");
      config.setViewUp(fromString(subsubNode.getValue()));
      pModel->setCameraConfiguration(config);      
    }
    return true;
  }

  /**
    apparently no legacy file with Mukpaths / PathCollections exists for this version, so only reading of paths necessary (atm)
  */
  bool VisualizationModelLegacyIO::load_v_0_2_0_0(const XmlNode& root, MukScene* pScene, VisScene* dummyVisScene, VisualizationModel* pModel)
  {
    auto node = root.getChild("Obstacles");
    {
      auto ndObstacles = node.getChildren();
      for (auto& ndObj : ndObstacles)
      {
        auto tmp = ndObj.getChild("Name");
        const std::string key = tmp.getValue();
        auto pObj = dummyVisScene->getObstacle(key);
        LoadNode<VisualObject>::load(ndObj, pObj);
      }
    }
    // vtkWindow
    node = root.getChild("Visualization");
    {
      CameraConfiguration config;
      auto subNode = node.getChild("Camera");
      auto subsubNode = subNode.getChild("FocalPoint");
      config.setFocalPoint(fromString(subsubNode.getValue()));
      subsubNode = subNode.getChild("Position");
      config.setPosition(fromString(subsubNode.getValue()));
      subsubNode = subNode.getChild("ViewUp");
      config.setViewUp(fromString(subsubNode.getValue()));
      pModel->setCameraConfiguration(config);      
    }
    return true;
  }

  /**
  */
  bool VisualizationModelLegacyIO::load_v_0_3_0_0(const XmlNode& root, MukScene* pScene, VisScene* dummyVisScene, VisualizationModel* pModel)
  {
    auto node = root.getChild("Obstacles");
    {
      auto ndObstacles = node.getChildren();
      auto keys = pScene->getObstacleKeys();
      for (const auto& key : keys)
      {
        auto pObj = dummyVisScene->getObstacle(key);
        auto iter = std::find_if(ndObstacles.begin(), ndObstacles.end(), [&] (const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndObstacles.end())
        {
          LoadNode<VisualObject>::load(*iter, pObj);
        }
      }
    }
    node = root.getChild("PathCollections");
    {
      auto ndCollections = node.getChildren();
      auto keys = pScene->getPathKeys();
      for (const auto& key : keys)
      {
        auto pObj = dummyVisScene->getPathCollection(key);
        auto iter = std::find_if(ndCollections.begin(), ndCollections.end(), [&] (const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndCollections.end())
        {
          //auto ndWp = iter->getChild("Waypoints");
          //auto pWp = pObj->getWaypoints();
          //LoadNode<VisWaypoints>::load(ndWp, pWp);

          const size_t N = pScene->getPathCollection(key).getPaths().size();
          auto ndPaths = iter->getChild("MukPaths").getChildren();
          for (size_t i(0); i < N; ++i)
          {
            LoadNode<VisMukPath>::load(ndPaths[i], dummyVisScene->getPathCollection(key)->getMukPath(i));
          }
        }
      }
    }
    // vtkWindow
    node = root.getChild("Visualization");
    {
      CameraConfiguration config;
      auto subNode = node.getChild("Camera");
      auto subsubNode = subNode.getChild("FocalPoint");
      config.setFocalPoint(fromString(subsubNode.getValue()));
      subsubNode = subNode.getChild("Position");
      config.setPosition(fromString(subsubNode.getValue()));
      subsubNode = subNode.getChild("ViewUp");
      config.setViewUp(fromString(subsubNode.getValue()));
      pModel->setCameraConfiguration(config);      
    }
    return true;    
  }
}
}

namespace
{
  /**
  */
  template<>
  void LoadNode<VisualObject>::load(XmlNode& node, std::shared_ptr<VisualObject> pObj)
  {
    std::string val = node.getChild("Visibility").getValue();
    if (val == "false")
    {
      pObj->setVisibility(false);
    }
    else
    {
      pObj->setVisibility(true);
    }
    val = node.getChild("Opacity").getValue();
    pObj->setOpacity(std::stof(val));
    val = node.getChild("Color").getValue();
    pObj->setDefaultColor(::fromString(val.c_str()));
  }

  /**
  */
  template<>
  void LoadNode<VisMukPath>::load(XmlNode& node, std::shared_ptr<VisMukPath> pObj)
  {
    LoadNode<VisualObject>::load(node, pObj);
    std::string val = node.getChild("Topology").getValue();
    const auto iterBegin = VisMukPath::s_topologyStr;
    const auto iterEnd   = iterBegin + VisMukPath::N_Topologies;
    auto iter = std::find_if(iterBegin, iterEnd, [&] (const char* str)
    {
      return val == str;
    });
    if (iter==iterEnd)
    {
      iter = iterBegin;
    }
    int idx = std::distance(VisMukPath::s_topologyStr, iter);
    pObj->setTopology(VisMukPath::EnTopology(idx));
  }

  /**
  */
  template<>
  void LoadNode<VisWaypoints>::load(XmlNode& node, std::shared_ptr<VisWaypoints> pObj)
  {
    LoadNode<VisualObject>::load(node, pObj);
    std::string val = node.getChild("Topology").getValue();
    const auto iterBegin = VisWaypoints::s_topologyStr;
    const auto iterEnd   = iterBegin + VisWaypoints::N_Topologies;
    auto iter = std::find_if(VisWaypoints::s_topologyStr, iterEnd, [&] (const char* str)
    {
      return val == str;
    });
    if (iter==iterEnd)
    {
      iter = iterBegin;
    }
    int idx = std::distance(VisWaypoints::s_topologyStr, iter);
    pObj->setTopology(VisWaypoints::EnTopology(idx));
  }

  /**
  */
  Vec3d fromString(const char* string)
  {
    Vec3d result;
    std::string str(string);
    str.erase(std::remove_if(str.begin(), str.end(), [&] (const char c) 
    {
      return c=='(' || c==')' || c==',';
    }), str.end());
    std::istringstream iss(str);
    iss >> result.x() >> result.y() >> result.z();
    return result;    
  }

  /**
  */
  gris::muk::LibVersion getLibVersion(const std::vector<gris::XmlNode>& xmlNodes)
  {    
    auto iter = std::find_if(xmlNodes.begin(), xmlNodes.end(), [&] (const auto& node) { return std::string("version") == node.getName(); });    
    gris::muk::LibVersion libVersion;
    if (iter != xmlNodes.end() )
    { 
      libVersion = LibVersion(iter->getValue());
    }
    else
    {
      libVersion = LibVersion("0.0.0.0");
    }
    return libVersion;
  }
}