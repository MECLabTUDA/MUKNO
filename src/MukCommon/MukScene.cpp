#include "private/muk.pch"
#include "private/MukSceneLegacyIO.h"

#include "MukScene.h"

#include "MukException.h"
#include "CollisionDetectorKdTree.h"
#include "PlannerFactory.h"
#include "PrunerFactory.h"
#include "InterpolatorFactory.h"
#include "NavigatorFactory.h"
#include "StateRegionFactory.h"

#include "Bounds.h"
#include "version.h"
#include "PathCollection.h"
#include "MukObstacle.h"
#include "SystemCalibration.h"
#include "ICollisionDetector.h"
#include "IPathPlanner.h"
#include "IPathPruner.h"
#include "IInterpolator.h"
#include "INavigator.h"
#include "MukStringToolkit.h"
#include "StateRegionSingleDirection.h"

#include "mukIO.h"
#include "version.h"

#include "gstd/XmlDocument.h"
#include "gstd/XmlNode.h"

#include <vtkObject.h>
#include <vtkPolyData.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/unique_ptr.hpp>

namespace
{
  using namespace gris;
  using namespace gris::muk;
  namespace fs = boost::filesystem;

  void load(const XmlNode& node, gstd::DynamicProperty& prop);
  void save(XmlNode& node, const gstd::DynamicProperty& prop);

  void saveProblemDefinition(const MukProblemDefinition& obj, XmlNode& node,
    const fs::path& rootDir, const fs::path& subDir, const std::string& pathCollName);
  void loadProblemDefinition(MukProblemDefinition& obj, const XmlNode& node, const fs::path& rootDir);
}

namespace gris
{
namespace muk
{
  /**
  */
  MukScene::MukScene()
    : mLocalBasePath("")
  {
    reset();
  }

  /**
  */
  MukScene::~MukScene()
  {
  }
  
  /**
  */
  void MukScene::setName(const std::string& name)
  {
    if (name.empty())
    {
      throw MUK_EXCEPTION_SIMPLE("Name of a Scene must not be empty (=\"\")!'");
    }   
    mSceneName = name;
  }

  /**
  */
  void MukScene::setPlanner(std::unique_ptr<IPathPlanner> pObj)
  {
    mpPlanner = std::move(pObj);
    mpPlanner ->setCollisionDetector(mpCollisionDetector);
  }

  /**
  */
  void MukScene::setPruner(std::unique_ptr<IPathPruner> pObj)
  {
    mpPruner = std::move(pObj);
    mpPruner->setCollisionDetector(mpCollisionDetector);
  }

  /**
  */
  bool MukScene::hasPathKey(const std::string& key) const
  {
    auto iter = std::find_if(mPaths.begin(), mPaths.end(), [&] (const auto& pPath) { return 0 == key.compare(pPath->getName()); });
    if (iter==mPaths.end())
      return false;
    else
      return true;
  }

  /**
  */
  std::vector<std::string> MukScene::getPathKeys() const
  {
    std::vector<std::string> keys;
    std::for_each(mPaths.begin(), mPaths.end(), [&] (const auto& pPath) { keys.push_back(pPath->getName()); } );
    return keys;
  }
  
  /**
  */
  std::vector<std::string> MukScene::getObstacleKeys() const
  {
    std::vector<std::string> keys;
    std::for_each(mObstacles.begin(), mObstacles.end(), [&] (const auto& pObs)  { keys.push_back(pObs->getName()); } );
    return keys;
  }

  /**
  */
  PathCollection& MukScene::getPathCollection(const std::string& key)
  {
    auto iter = std::find_if(mPaths.begin(), mPaths.end(), [&] (const auto& pObj)
    {
      return 0 == key.compare(pObj->getName());
    });
    if (iter==mPaths.end())
      throw MUK_EXCEPTION("no collection found with this key", key.c_str());    
    return **iter;
  }
  
  /**
  */
  void MukScene::setCollisionDetector(std::shared_ptr<ICollisionDetector> pObj)
  {
    mpCollisionDetector = pObj; mpPlanner->setCollisionDetector(pObj); mpPruner->setCollisionDetector(pObj); 
  }

  /** \brief creates all necessary and dependent structures for a path

    Waypoints, MukPath and Visualization will be created
  */
  void MukScene::insertPathCollection(const std::string& key)
  {
    if (hasPathKey(key))
      throw MUK_EXCEPTION("Key already exists", key.c_str());

    mPaths.push_back(std::make_shared<PathCollection>(key));
  }

  /**
  */
  void MukScene::deletePathCollection(const std::string& key)
  { 
    mPaths.erase(
      std::remove_if(mPaths.begin(), mPaths.end(), [&] (const auto& coll) { return 0 == key.compare(coll->getName()); }),
      mPaths.end());
  }

  /**
  */
  std::shared_ptr<MukObstacle> MukScene::getObstacle(const std::string& key) const
  {
    auto iter = std::find_if(mObstacles.begin(), mObstacles.end(), [&] (const auto& pObs) { return key == pObs->getName(); });
    if (iter==mObstacles.end())
      throw MUK_EXCEPTION("obstacle does not exist", key.c_str());
    return *iter;
  }

  void MukScene::insertObstacle(std::shared_ptr<MukObstacle> obs)
  {
    mObstacles.push_back(obs);
    mpCollisionDetector->addObstacle(obs->getName(), obs->getData());
    mpCollisionDetector->setActive(obs->getName(), obs->getActive());
  }

  void MukScene::deleteObstacle(const std::string& key)
  {
    mObstacles.erase(
      std::remove_if(mObstacles.begin(), mObstacles.end(), [&] (const auto& pObs) { return key == pObs->getName(); }),
      mObstacles.end());
    mpCollisionDetector->removeObstacle(key);
  }

  void MukScene::setObstacleActive(const std::string& key, bool on)
  {
    auto iter = std::find_if(mObstacles.begin(), mObstacles.end(), [&] (const auto& pObs) { return key == pObs->getName(); });
    if (iter==mObstacles.end())
      throw MUK_EXCEPTION("obstacle does not exist", key.c_str());
    (*iter)->setActive(on);
    mpCollisionDetector->setActive(key, on);
  }

  /**
  */
  void MukScene::save(const std::string& fullSceneFileName_) const
  {
    namespace fs = boost::filesystem;
    fs::path fullSceneFileName = fullSceneFileName_;
    if ( ! fullSceneFileName.has_extension())
      throw MUK_EXCEPTION("no valid filename specified (need some extension e.g. '.mukscene')", fullSceneFileName_.c_str());
    fullSceneFileName.replace_extension(".mukscene");    
    fs::path dir    = fullSceneFileName.parent_path();
    fs::path subdir = fullSceneFileName.stem();
    if (! fs::is_directory(dir / subdir))
      fs::create_directory(dir / subdir);
    
    auto doc = gris::XmlDocument::create("Mukno");
    gris::XmlNode root  = doc->getRoot();
    // version
    gris::XmlNode node  = root.addChild("version");
    {
      DWORD major, minor, revision, build;
      getMuknoLibraryVersion("MukCommon", major, minor, revision, build);
      const std::string version = (boost::format("%d.%d.%d.%d") % major % minor % revision % build).str();
      node.setValue(version.c_str());
    }
    node  = root.addChild("SurgeryPlanning").addChild("MukScene").addChild("SceneName");
    {
      node.setValue(mSceneName.c_str());
    }
    node  = root.getChild("SurgeryPlanning").getChild("MukScene").addChild("Obstacles");
    // obstacles    
    for (const auto& pObj : mObstacles)
    {
      gris::XmlNode next = node.addChild("Obstacle");      
      next.addChild("Name").setValue(pObj->getName().c_str());
      next.addChild("File").setValue(pObj->getFileName().c_str());
      if (pObj->getActive())
        next.addChild("Active").setValue("true");
      else
        next.addChild("Active").setValue("false");
    }
    // paths
    node = root.getChild("SurgeryPlanning").getChild("MukScene").addChild("PathCollections");    
    for (const auto& pColl : mPaths)
    {
      auto next = node.addChild("PathCollection");
      auto tmp  = next.addChild("Name");
      tmp.setValue(pColl->getName().c_str());
      tmp  = next.addChild("InactiveObstacles");
      const auto& obs = pColl->getObstacles();
      for (size_t i(0); i<obs.size(); ++i)
      {
        tmp.addChild("Obstacle").setValue(obs[i].c_str());
      }
      tmp = next.addChild("ProblemDefinition");
      {
        ::saveProblemDefinition(*pColl->getProblemDefinition(), tmp, dir, subdir, pColl->getName());
      }
      tmp = next.addChild("MukPaths");
      {
        for(size_t i(0); i<pColl->getPaths().size(); ++i)
        {
          const auto& path = pColl->getPaths()[i];
          auto filename = subdir / (boost::format("Path_%s_%d.txt") % pColl->getName().c_str() % i).str();
          auto full     = dir / filename;
          std::ofstream ofs(full.string());
          boost::archive::text_oarchive oa(ofs);
          oa << path;
          auto pathNode = tmp.addChild("MukPath");
          pathNode.addChild("File").setValue(filename.generic_string().c_str());
        }        
      }
    }
    auto baseNode = root.getChild("SurgeryPlanning").addChild("Configuration").addChild("Planning");
    // planner
    {
      node = baseNode.addChild("Planner");
      auto tmp = node.addChild("Type");
      tmp.setValue( mpPlanner->name() );
      std::vector<std::string> props;
      mpPlanner->getPropertyNames(props);
      for(const auto& prop : props)
      {
        tmp = node.addChild(prop.c_str());
        std::string str;
        mpPlanner->getProperty(prop, str);
        tmp.setValue(str.c_str());
      }

    }
    // pruner
    {
      node = baseNode.addChild("Pruner");
      auto tmp = node.addChild("Type");
      tmp.setValue( mpPruner->name() );
      tmp = node.addChild("MaxStepSize");
      tmp.setValue( (boost::format("%f") % mpPruner->getMaxStepSize() ).str().c_str() );
      tmp = node.addChild("MaxDistance");
      tmp.setValue( (boost::format("%f") % mpPruner->getMaxDistance() ).str().c_str() );
    }
    // interpolator
    {
      node = baseNode.addChild("Interpolator");
      auto tmp = node.addChild("Type");
      tmp.setValue( mpInterpolator->name() );
      tmp = node.addChild("InterpolationType");
      tmp.setValue( IInterpolator::InterpolationTypesAsStrings[mpInterpolator->getInterpolationType()] );
      tmp = node.addChild("PointsPerSegment");
      tmp.setValue( (boost::format("%d") % mpInterpolator->getPointsPerSegment() ).str().c_str() );
      tmp = node.addChild("Resolution");
      tmp.setValue( (boost::format("%f") % mpInterpolator->getResolution() ).str().c_str() );      
    }    
    gris::XmlDocument::save( fullSceneFileName.string().c_str() , *doc);
  }

  /**
  */
  void MukScene::load(const std::string& filename)
  {
    namespace fs = boost::filesystem;
    fs::path p = filename;
    if (!fs::is_regular_file(filename))
      throw MUK_EXCEPTION("File does not exist!", p.string().c_str());

    // holds the new data -> swap later with this
    auto dummyScene = std::make_unique<MukScene>();
    dummyScene->setLocalBasePath(mLocalBasePath); //mLocalBasePath

    auto doc = gris::XmlDocument::read(filename.c_str());
    gris::XmlNode root = doc->getRoot();

    if (MukSceneLegacyIO::load(filename, root, dummyScene.get()))
    {
      LOG_LINE << "Loaded a legacy file";
    }
    else
    {
      auto sceneNode = root.getChild("SurgeryPlanning").getChild("MukScene");
      auto node = sceneNode.getChild("SceneName");
      {
        dummyScene->mSceneName = node.getValue();
      }
      node = sceneNode.getChild("Obstacles");
      {
        auto obstacles = node.getChildren();
        for (auto& obj : obstacles)
        {
          auto pObj = std::make_shared<MukObstacle>();
          pObj->setName(obj.getChild("Name").getValue());
          const auto* fn = obj.getChild("File").getValue();
          if (fs::path(fn).is_absolute())
          {
            pObj->setFileName(fn);
          }
          else
          {
            auto file_name = fs::path(dummyScene->mLocalBasePath) / fn;
            pObj->setFileName(file_name.generic_string());
          }
          pObj->load();
          auto tmp = obj.getChild("Active");
          if (std::string(tmp.getValue()) == "true")
          {
            pObj->setActive(true);
          }
          else
          {
            pObj->setActive(false);
          }
          dummyScene->insertObstacle(pObj);
        }
      }
      node = sceneNode.getChild("PathCollections");
      {
        auto ndCollections = node.getChildren();
        for (auto& ndColl : ndCollections)
        {
          const std::string key = ndColl.getChild("Name").getValue();
          dummyScene->insertPathCollection(key);
          auto& coll = dummyScene->getPathCollection(key);
          ::load(ndColl, coll);
          {
            const auto* key = "InactiveObstacles";
            if (ndColl.hasChild(key))
            {
              auto ndObs = ndColl.getChild(key).getChildren();
              for (const auto& node : ndObs)
                coll.addObstacle(node.getValue());
            }
          }
          auto pProbDef = coll.getProblemDefinition();
          auto ndProbDef = ndColl.getChild("ProblemDefinition");
          loadProblemDefinition(*pProbDef, ndProbDef, fs::path(filename).parent_path());
          auto tmp = ndColl.getChild("MukPaths");
          auto ndMukPaths = tmp.getChildren();
          for (const auto& ndPath : ndMukPaths)
          {
            const fs::path fn = fs::path(filename).parent_path() / ndPath.getChild("File").getValue();
            MukPath path;
            if (!fs::is_regular_file(fn))
            {
              LOG_LINE << __FUNCTION__ << "   Path: " << key << "   File does not exist: " << fn.string();
              dummyScene->deletePathCollection(key);
              continue;
            }
            std::ifstream ifs(fn.string());
            boost::archive::text_iarchive ia(ifs);
            ia >> path;
            coll.insertPath(path);
          }
        }
      }
      gris::XmlNode configNode = root.getChild("SurgeryPlanning").getChild("Configuration");
      node = configNode.getChild("Planning").getChild("Planner");
      {
        auto tmp = node.getChild("Type");
        std::vector<std::string> keys;
        GetPlannerFactory().getKeys(keys);
        if (std::any_of(keys.begin(), keys.end(), [&] (const std::string& str) { return str == tmp.getValue(); }))
        {
          auto planner = GetPlannerFactory().create(tmp.getValue());
          ::load(node, *planner);              
          dummyScene->setPlanner(std::move(planner));
        }
      }
      node = configNode.getChild("Planning").getChild("Pruner");
      {
        auto tmp = node.getChild("Type");
        auto pruner = GetPrunerFactory().create(tmp.getValue());
        ::load(node, *pruner);
        dummyScene->setPruner(std::move(pruner));
      }
      node = configNode.getChild("Planning").getChild("Interpolator");
      {
        auto tmp = node.getChild("Type");
        auto interpolator = GetInterpolatorFactory().create(tmp.getValue());
        ::load(node, *interpolator);
        dummyScene->setInterpolator(std::move(interpolator));
      }
    }
    swap(*dummyScene.get());    
  }

  /**
  */
  void MukScene::swap(MukScene& o)
  {
    mSceneName.swap(o.mSceneName);
    mpSystemCalibration.swap(o.mpSystemCalibration);
    mpCollisionDetector.swap(o.mpCollisionDetector);
    mObstacles.swap(o.mObstacles);
        
    mpPlanner.swap(o.mpPlanner);
    mpInterpolator.swap(o.mpInterpolator);
    mpPruner.swap(o.mpPruner);

    mPaths.swap(o.mPaths);
  }

  void MukScene::reset()
  {
    auto ctCalib = CTCalibration();
    ctCalib.setDimSize( Vec3d(512, 512, 132) );
    ctCalib.setElementSpacing( Vec3d(0.179688, 0.179688, 0.399994) );
    mpSystemCalibration = std::make_shared<SystemCalibration>();
    mpSystemCalibration->setCtCalibration(ctCalib);
    if (nullptr == mpCollisionDetector.get())
    {
      mpCollisionDetector = std::make_shared<CollisionDetectorKdTree>();
    }
    else
    {
      mpCollisionDetector->clear();
    }     
    mPaths.clear();
    mObstacles.clear();
    mSceneName.clear();
    mpPlanner      = nullptr == mpPlanner.get()       ? GetPlannerFactory().create("ManualPlanner") : GetPlannerFactory().create(mpPlanner->name());
    mpPruner       = nullptr == mpPruner.get()        ? GetPrunerFactory().create("Yang-Spiral-Pruner") : GetPrunerFactory().create(mpPruner->name());
    mpInterpolator = nullptr == mpInterpolator.get()  ? GetInterpolatorFactory().create("Linear-Interpolator") : GetInterpolatorFactory().create(mpInterpolator->name());    

    mpPlanner->setCollisionDetector(mpCollisionDetector);
    mpPruner ->setCollisionDetector(mpCollisionDetector);
  }
}
}

namespace
{
  /**
  */
  void load(const XmlNode& node, gstd::DynamicProperty& prop)
  {
    std::vector<std::string> names;
    prop.getPropertyNames(names);
    for (const auto& name : names)
    {
      if (const_cast<XmlNode&>(node).hasChild(name.c_str())) // bug in gstd
      {
        prop.setProperty(name, node.getChild(name.c_str()).getValue());
      }
    }
  }

  /**
  */
  void save(XmlNode& node, const gstd::DynamicProperty& prop)
  {
    std::vector<std::string> names;
    prop.getPropertyNames(names);
    for (const auto& name : names)
    {
      std::string value;
      prop.getProperty(name, value);
      node.addChild(name.c_str()).setValue(value.c_str());
    }
  }

  /**
  */
  void saveProblemDefinition(const MukProblemDefinition& obj, XmlNode& node, 
    const fs::path& rootDir,
    const fs::path& subDir,
    const std::string& pathCollName)
  {
    ::save(node, obj);

    namespace fs = boost::filesystem;
    auto filename = subDir / (boost::format("ProblemDefinition_%s.txt") % pathCollName).str();
    auto full = rootDir / filename;
    std::ofstream ofs(full.generic_string().c_str());
    obj.save(full.generic_string(), 0);
    node.addChild("FileName").setValue(filename.generic_string().c_str());
    // finished principally, but add properties to xml for visual inspection and easier alterations
    auto subNode = node.addChild("StartRegions");
    for (size_t i(0); i < obj.sizeStart(); ++i)
    {
      const std::string regionName = "StartRegion" + std::to_string(i);
      auto ndRegion = subNode.addChild(regionName.c_str() );
      ndRegion.addChild("ClassName").setValue(obj.getStartRegion(i).name());
      ::save(ndRegion, obj.getStartRegion(i));
    }
    subNode = node.addChild("GoalRegions");
    for (size_t i(0); i < obj.sizeGoal(); ++i)
    {
      const std::string regionName = "GoalRegion" + std::to_string(i);
      auto ndRegion = subNode.addChild(regionName.c_str());
      ndRegion.addChild("ClassName").setValue(obj.getGoalRegion(i).name());
      ::save(ndRegion, obj.getGoalRegion(i));
    }
    subNode = node.addChild("Waypoints");
    {
      const std::string regionName = "Waypoints";
      //auto ndRegion = subNode.addChild(regionName.c_str());
      //ndRegion.addChild("ClassName").setValue(obj.getWaypoint(i));
    }
  }

  /**
  */
  void loadProblemDefinition(MukProblemDefinition& obj, const XmlNode& node, const fs::path& rootDir)
  {
    auto filename =  rootDir / node.getChild("FileName").getValue();
    if (!fs::is_regular_file(filename))
    {
      throw MUK_EXCEPTION("File not found", filename.generic_string().c_str());
    }
    try
    {
      obj.load(filename.generic_string(), 0);
    }
    catch (boost::archive::archive_exception& e)
    {
      LOG_LINE << "internal error: " << e.what();
      throw MUK_EXCEPTION("Failed to load Problem Definition. Incompatible archive version.", filename.generic_string().c_str());
    }
    // inconsistent save/load:  serializing with boost works only with boosts's onw factory.
    // easy manipulation works with xml
    // -> first load with boost, then update possible manual changes in xml
    ::load(node, obj);
    auto ndRegions = node.getChild("StartRegions");
    auto ndChildren = ndRegions.getChildren();
    for (size_t i(0); i<ndChildren.size(); ++i)
    {
      auto& nd = ndChildren[i];
      ::load(nd, obj.getStartRegion(i));
    }
    ndRegions = node.getChild("GoalRegions");
    ndChildren = ndRegions.getChildren();
    for (size_t i(0); i<ndChildren.size(); ++i)
    {
      auto& nd = ndChildren[i];
      ::load(nd, obj.getGoalRegion(i));
    }
    ndRegions = node.getChild("Waypoints");
    {
      /*   auto fileName = fs::path(dummyScene->mLocalBasePath) / node.getChild("FileName").getValue();
      std::ifstream ifs(fileName.generic_string());
      boost::archive::text_iarchive ia(ifs);
      pProbDef->insertWaypoint(wp);*/
    }
  }
}