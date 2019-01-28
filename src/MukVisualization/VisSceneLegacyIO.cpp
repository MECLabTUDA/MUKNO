#include "private/muk.pch"
#include "private/VisSceneLegacyIO.h"

#include "VisPathCollection.h"
#include "VisObstacle.h"
#include "VisScene.h"

#include "MukCommon/MukException.h"
#include "MukCommon/version.h"

#include "gstd/DynamicProperty.h"
#include "gstd/XmlNode.h"

#include <boost/filesystem.hpp>

#include <string>

namespace
{
  namespace fs = boost::filesystem;
  // searches for node <version> ... </version> and returns the LibVersion (converted from the corresponding string) from it
  gris::muk::LibVersion getLibVersion(const std::vector<gris::XmlNode>& xmlNodes);

  using namespace gris;
  using namespace gris::muk;

  template<class T>
  struct LoadNode
  {
    static void load_v_0_4_1_0(const XmlNode& node, std::shared_ptr<T> pObj);
    static void load_v_0_5    (const XmlNode& node, std::shared_ptr<T> pObj);
  };

  void load(const XmlNode& node, gstd::DynamicProperty& prop);
}


namespace gris
{
namespace muk
{
  /**
  */
  bool VisSceneLegacyIO::load(const std::string& filename, const XmlNode& root, VisScene* pScene)
  {
    auto children = root.getChildren();
    // version 
    auto libVersion = ::getLibVersion(children);

    if (libVersion < LibVersion(0, 4, 1, 0))
    {
      std::stringstream ss;
      ss << "found version: " << libVersion.mMajor << " " << libVersion.mMinor << " " << libVersion.mRevision << " " << libVersion.mBuild;
      throw MUK_EXCEPTION_SIMPLE("Unable to load visscene-file. \nThis variant of saving a scene is no longer supported.");
    }
    if (libVersion < LibVersion(0, 4, 2, 0))
    {
      load_v_0_4_1_0(filename, root, pScene);
      return true;
    }
    if (libVersion < LibVersion(0, 6, 0, 0))
    {
      load_v_0_5(filename, root, pScene);
      return true;
    }
    return false;
  }

  /**
  */
  void VisSceneLegacyIO::load_v_0_5(const std::string& filename, const XmlNode& root, VisScene* pScene)
  {
    auto sceneNode = root.getChild("SurgeryPlanning").getChild("MukScene");
    auto node = sceneNode.getChild("Obstacles");
    {
      auto ndObstacles = node.getChildren();
      auto keys = pScene->getObstacleKeys();
      for (const auto& key : keys)
      {
        auto pObj = pScene->getObstacle(key);
        auto iter = std::find_if(ndObstacles.begin(), ndObstacles.end(), [&] (const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndObstacles.end())
        {
          ::load(*iter, *pObj);
        }
      }
    }
    node = sceneNode.getChild("PathCollections");
    {
      auto ndCollections = node.getChildren();
      auto keys = pScene->getPathCollectionKeys();
      for (const auto& key : keys)
      {
        auto pObj = pScene->getPathCollection(key);
        auto iter = std::find_if(ndCollections.begin(), ndCollections.end(), [&] (const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndCollections.end())
        {
          auto pColl = pScene->getPathCollection(key);
          ::LoadNode<VisPathCollection>::load_v_0_5(*iter, pColl);
        }
      }
    }
  }

  /**
  */
  void VisSceneLegacyIO::load_v_0_4_1_0(const std::string& filename, const XmlNode& root, VisScene* pScene)
  {
    auto sceneNode = root.getChild("SurgeryPlanning").getChild("MukScene");
    auto node = sceneNode.getChild("Obstacles");
    {
      auto ndObstacles = node.getChildren();
      auto keys = pScene->getObstacleKeys();
      for (const auto& key : keys)
      {
        auto pObj = pScene->getObstacle(key);
        auto iter = std::find_if(ndObstacles.begin(), ndObstacles.end(), [&](const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndObstacles.end())
        {
          ::load(*iter, *pObj);
        }
      }
    }
    node = sceneNode.getChild("PathCollections");
    {
      auto ndCollections = node.getChildren();
      auto keys = pScene->getPathCollectionKeys();
      for (const auto& key : keys)
      {
        auto pObj = pScene->getPathCollection(key);
        auto iter = std::find_if(ndCollections.begin(), ndCollections.end(), [&](const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndCollections.end())
        {
          auto pColl = pScene->getPathCollection(key);
          ::LoadNode<VisPathCollection>::load_v_0_4_1_0(*iter, pColl);
        }
      }
    }
  }
}
}

namespace
{
  using namespace gris::muk;

  /**
  */
  gris::muk::LibVersion getLibVersion(const std::vector<gris::XmlNode>& xmlNodes)
  {
    auto iter = std::find_if(xmlNodes.begin(), xmlNodes.end(), [&](const auto& node) { return std::string("version") == node.getName(); });
    gris::muk::LibVersion libVersion;
    if (iter != xmlNodes.end())
    {
      libVersion = LibVersion(iter->getValue());
    }
    else
    {
      libVersion = LibVersion("0.0.0.0");
    }
    return libVersion;
  }
  
  /**
  */
  void load(const XmlNode& node, gstd::DynamicProperty& prop)
  {
    std::vector<std::string> propNames;
    prop.getPropertyNames(propNames);
    for (const auto& name : propNames)
    {
      if (node.hasChild(name.c_str()))
      {
        std::string value = node.getChild(name.c_str()).getValue();
        if (name == "Color")
        {
          value.erase(std::remove_if(value.begin(), value.end(), [&] (const char c)
          {
            return c == '(' || c == ')' || c == ',';
          }), value.end());
        }
        prop.setProperty(name, value);
      }
    }
  }

  /**
  */
  template<>
  void LoadNode<VisPathCollection>::load_v_0_4_1_0(const XmlNode& node, std::shared_ptr<VisPathCollection> pObj)
  {
    auto N = pObj->sizeStart();
    auto ndRegions = node.getChild("Waypoints");
    // do nothing ?
    
    auto ndPaths = node.getChild("MukPaths").getChildren();
    if (!ndPaths.empty())
    {
      int i(0);
      for (auto& node : ndPaths)
      {
        auto pVis = pObj->getMukPath(i);
        ::load(node, *pObj->getMukPath(i));
        ++i;
      }
    }
  }

  /**
  */
  template<>
  void LoadNode<VisPathCollection>::load_v_0_5(const XmlNode& node, std::shared_ptr<VisPathCollection> pObj)
  {
    auto N = pObj->sizeStart();
    auto ndProbDef = node.getChild("ProblemDefinition");
    auto ndRegions = ndProbDef.getChild("StartRegions");
    {
      for (size_t i(0); i<N; ++i)
      {
        auto nodeName = "StartRegion" + std::to_string(i);
        ::load(ndRegions.getChild(nodeName.c_str()), pObj->getStartRegion(i));
      }
    }
    N = pObj->sizeGoal();
    ndRegions = ndProbDef.getChild("GoalRegions");
    {
      for (size_t i(0); i<N; ++i)
      {
        auto nodeName = "GoalRegion" + std::to_string(i);
        ::load(ndRegions.getChild(nodeName.c_str()), pObj->getGoalRegion(i));
      }
    }

    auto ndPaths = node.getChild("MukPaths").getChildren();
    if (!ndPaths.empty())
    {
      int i(0);
      for (auto& node : ndPaths)
      {
        auto pVis = pObj->getMukPath(i);
        ::load(node, *pObj->getMukPath(i));
        ++i;
      }
    }
  }

  /**
  */
  Vec3d fromString(const char* string)
  {
    Vec3d result;
    std::string str(string);
    str.erase(std::remove_if(str.begin(), str.end(), [&](const char c)
    {
      return c == '(' || c == ')' || c == ',';
    }), str.end());
    std::istringstream iss(str);
    iss >> result.x() >> result.y() >> result.z();
    return result;
  }
}