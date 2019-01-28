#include "private/muk.pch"
#include "private/VisSceneLegacyIO.h"

#include "VisScene.h"
#include "VisPathCollection.h"
#include "VisObstacle.h"
#include "VisAbstractObject.h"
#include "IVisRobot.h"

#include "MukCommon\MukScene.h"
#include "MukCommon/MukStringToolkit.h"
#include "MukCommon/MukException.h"

#include "gstd/XmlDocument.h"
#include "gstd/XmlNode.h"

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

namespace
{
  using namespace gris;
  using namespace gris::muk;

  template<class T>
  struct SaveToNode
  {
    static void save(XmlNode& node, std::shared_ptr<T> pObj);
  };
  template<class T>
  struct LoadNode
  {
    static void load(const XmlNode& node, std::shared_ptr<T> pObj);
  };

  void save(XmlNode& node,        const gstd::DynamicProperty& prop);
  void load(const XmlNode& node,        gstd::DynamicProperty& prop);
}

namespace gris
{
namespace muk
{
  /**
  */
  VisScene::~VisScene()
  {
  }

  /** \brief Overrides an existing path collection
  */
  void VisScene::setPathCollection(std::shared_ptr<VisPathCollection>& obj)
  {
    auto iter = retrieveName(mPaths.begin(), mPaths.end(), obj->getName());
    if (iter == mPaths.end())
    {
      throw MUK_EXCEPTION("Could not find name", obj->getName().c_str());
    }
    iter->swap(obj);
  }

  /**
  */
  std::shared_ptr<VisPathCollection> VisScene::getPathCollection(const std::string& name) const
  {
    auto iter = retrieveName(mPaths.begin(), mPaths.end(), name);
    if (iter == mPaths.end())
    {
      throw MUK_EXCEPTION("Could not find name", name.c_str());
    }
    return *iter;
  }

  /**
  */
  bool VisScene::hasPathCollection(const std::string& name) const
  {
    auto iter = retrieveName(mPaths.begin(), mPaths.end(), name);
    if (iter == mPaths.end())
      return false;
    else
      return true;
  }

  /**
  */
  std::shared_ptr<VisObstacle> VisScene::getObstacle(const std::string& name) const
  {
    auto iter = find_if(mObstacles.begin(), mObstacles.end(), [&] (const auto& pObj) { return name == pObj->getName(); });
    if (iter==mObstacles.end())
      return std::shared_ptr<VisObstacle>();
    else
      return *iter;
  }

  /**
  */
  std::shared_ptr<VisAbstractObject> VisScene::getObject(const std::string& name) const
  {
    auto iter = find_if(mObjects.begin(), mObjects.end(), [&] (const auto& pObj) { return name == pObj->getName(); });
    if (iter==mObjects.end())
    {
      return std::shared_ptr<VisAbstractObject>();      
    }
    else
      return *iter;
  }

  /**
  */
  void VisScene::addPathCollection(std::shared_ptr<VisPathCollection> pObj)
  {
    mPaths.push_back(pObj);
  }

  /**
  */
  void VisScene::addObstacle(std::shared_ptr<VisObstacle> pObj)
  {
    mObstacles.push_back(pObj);
  }

  /**
  */
  void VisScene::addObject(std::shared_ptr<VisAbstractObject> pObj)
  {
    mObjects.push_back(pObj);
  }

  /**
  */
  void VisScene::deletePathCollection(const std::string& name)
  {
    auto iter = retrieveName(mPaths.begin(), mPaths.end(), name);
    if (iter == mPaths.end())
    {
      throw MUK_EXCEPTION("Could not find name", name.c_str());
    }
    mPaths.erase(iter);   
  }

  /**
  */
  void VisScene::deleteObstacle(const std::string& name)
  {
    auto iter = find_if(mObstacles.begin(), mObstacles.end(), [&] (const auto pObj) { return name == pObj->getName(); } );
    if (iter==mObstacles.end())
    {
      throw MUK_EXCEPTION("Visualization of Obstacle does not exist", (boost::format("Obstacle name: %s") % name).str().c_str());
    }
    mObstacles.erase(iter);
  }

  /**
  */
  void VisScene::deleteObject(const std::string& name)
  {
    auto iter = find_if(mObjects.begin(), mObjects.end(), [&] (const auto pObj) { return name == pObj->getName(); } );
    if (iter==mObjects.end())
    {
      throw MUK_EXCEPTION("Visualization of Object does not exist", (boost::format("Object name: %s") % name).str().c_str());
    }
    mObjects.erase(iter);    
  }

  /**
  */
  bool VisScene::hasObject(const std::string& name)
  {
    auto iter = std::find_if(mObjects.begin(), mObjects.end(), [&] (const auto pObj) { return name == pObj->getName(); } );
    if (iter==mObjects.end())
    {
      return false;
    }
    return true;
  }

  /**
  */
  void VisScene::reset()
  {
    mPaths.clear();
    mObstacles.clear();;
    mObjects.clear();;
  }

  /**
  */
  std::vector<std::string> VisScene::getAbstractObjectKeys() const
  {
    std::vector<std::string> keys;
    for (const auto& pObj : mObjects)   
    {
      keys.push_back(pObj->getName());
    }
    return keys;
  }

  /**
  */
  void VisScene::setRobot(std::shared_ptr<IVisRobot> pObj)
  {
    mpVisRobot = pObj;
  }

  IVisRobot& VisScene::getRobot()
  {
    if (nullptr == mpVisRobot.get())
    {
      throw MUK_EXCEPTION_SIMPLE("robot visualization not set yet");
    }
    return *mpVisRobot;
  }

  /**
  */
  std::vector<std::string> VisScene::getPathCollectionKeys() const
  {
    std::vector<std::string> keys;
    for (const auto& pObj : mPaths)   
    {
      keys.push_back(pObj->getName());
    }
    return keys;
  }
  
  /**
  */
  std::vector<std::string> VisScene::getObstacleKeys() const
  {
    std::vector<std::string> keys;
    for (const auto& pObj : mObstacles)   
    {
      keys.push_back(pObj->getName());
    }
    return keys;
  }

  /** \brief requires an already existing Xml-File created by MukScene->save(...)
  */
  void VisScene::save(const std::string& filename_) const
  {
    namespace fs = boost::filesystem;
    fs::path filename (filename_);
    if (!fs::is_regular_file(filename))
    {
      throw MUK_EXCEPTION("Could not save VisScene", (boost::format("File does not exist: %s") % filename_).str().c_str());
    }
    auto pDoc = gris::XmlDocument::read(filename_.c_str());
    gris::XmlNode root = pDoc->getRoot();
    auto node = root.getChild("SurgeryPlanning").getChild("MukScene").getChild("Obstacles");
    {
      auto keys = this->getObstacleKeys();
      auto ndObs = node.getChildren();
      for (const auto& key : keys)
      {
        auto pObj = this->getObstacle(key);
        auto iter = std::find_if(ndObs.begin(), ndObs.end(), [&] (const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndObs.end())
        {
          ::save(*iter, *pObj);
        }
        else
        {
          LOG_LINE << "Warning: could not find Visualization of Obstacle " << key;
        }
      }
    }
    node = root.getChild("SurgeryPlanning").getChild("MukScene").getChild("PathCollections");
    {
      auto keys  = getPathCollectionKeys();
      auto ndColls = node.getChildren();
      for (const auto& key : keys)
      {
        auto pColl = this->getPathCollection(key);
        auto iter = std::find_if(ndColls.begin(), ndColls.end(), [&] (const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndColls.end())
        {
          auto ndProbDef = iter->getChild("ProblemDefinition");
          {
            ::SaveToNode<VisPathCollection>::save(ndProbDef, pColl);
          }
          auto ndPaths = iter->getChild("MukPaths").getChildren();
          if ( ! ndPaths.empty())
          {
            int i(0);
            for (auto& node : ndPaths)
            {
              auto pVis = pColl->getMukPath(i);
              ::save(node, *pVis);
              ++i;
            }
          }
        }
      }
    }
    gris::XmlDocument::save(filename_.c_str(), *pDoc);
  }

  /**
  */
  void VisScene::load(const std::string& filename)
  {
    namespace fs = boost::filesystem;
    fs::path p = filename;
    if (!fs::is_regular_file(filename))
      throw MUK_EXCEPTION("File does not exist!", p.string().c_str());

    auto doc = gris::XmlDocument::read(filename.c_str());
    gris::XmlNode root = doc->getRoot();

    if (VisSceneLegacyIO::load(filename, root, this))
    {
      return;
    }

    auto sceneNode = root.getChild("SurgeryPlanning").getChild("MukScene");
    auto node = sceneNode.getChild("Obstacles");
    {
      auto ndObstacles = node.getChildren();
      auto keys = getObstacleKeys();
      for (const auto& key : keys)
      {
        auto pObj = getObstacle(key);
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
      auto keys = getPathCollectionKeys();
      for (const auto& key : keys)
      {
        auto pObj = getPathCollection(key);
        auto iter = std::find_if(ndCollections.begin(), ndCollections.end(), [&] (const auto& node) { return key == node.getChild("Name").getValue(); });
        if (iter != ndCollections.end())
        {
          auto pColl = getPathCollection(key);
          ::LoadNode<VisPathCollection>::load(*iter, pColl);
        }
      }
    }
  }
}
}


namespace
{
  /**
  */
  void save(XmlNode& node, const gstd::DynamicProperty& prop)
  {
    std::vector<std::string> propNames;
    prop.getPropertyNames(propNames);
    for (const auto& name : propNames)
    {
      std::string value;
      prop.getProperty(name, value);
      node.addChild(name.c_str()).setValue(value.c_str());
    }
  }
  
  /**
  */
  void load(const XmlNode& node, gstd::DynamicProperty& prop)
  {
    std::vector<std::string> propNames;
    prop.getPropertyNames(propNames);
    for (const auto& name : propNames)
    {
      if (const_cast<XmlNode&>(node).hasChild(name.c_str()))
        prop.setProperty(name, node.getChild(name.c_str()).getValue());
    }
  }
  
  /**
  */
  template<>
  void SaveToNode<VisPathCollection>::save(XmlNode& node, std::shared_ptr<VisPathCollection> pObj)
  {
    auto N = pObj->sizeStart();
    if (N != 0)
    {
      auto ndRegions = node.getChild("StartRegions");
      {
        for (size_t i(0); i < N; ++i)
        {
          auto nodeName = "StartRegion" + std::to_string(i);
          if (ndRegions.hasChild(nodeName.c_str())) // doesn't exist, if a state region was created in the visualization window but not yet included in the actual problem definition
          {
            ::save(ndRegions.getChild(nodeName.c_str()), pObj->getStartRegion(i));
          }
          else
          {
            LOG_LINE << "Start region " << nodeName << " is not yet part of the internal ProblemDefinition.\n Update Waypoints and save again?";
          }
        }
      }
    }
    N = pObj->sizeGoal();
    if (N != 0)
    {
      auto ndRegions = node.getChild("GoalRegions");
      {
        for (size_t i(0); i < N; ++i)
        {
          auto nodeName = "GoalRegion" + std::to_string(i);
          if (ndRegions.hasChild(nodeName.c_str())) // doesn't exist, if a state region was created in the visualization window but not yet included in the actual problem definition
          {
            ::save(ndRegions.getChild(nodeName.c_str()), pObj->getGoalRegion(i));
          }
          else
          {
            LOG_LINE << "Goal region " << nodeName << " is not yet part of the internal ProblemDefinition.\n Update Waypoints and save again?";
          }
        }
      }
    }
    if (node.hasChild("Waypoints"))
    {
      auto ndRegions = node.getChild("Waypoints");
      {
        // buggy ProblemDefinition has vector of MukStates, Visualization has vector of VisStateRegions
        //::save(ndRegions, pObj->getWaypoint(i));
      }
    }
  }

  /**
  */
  template<>
  void LoadNode<VisPathCollection>::load(const XmlNode& node, std::shared_ptr<VisPathCollection> pObj)
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
}
