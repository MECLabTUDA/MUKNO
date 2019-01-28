#include "private/muk.pch"
#include "private/MukSceneLegacyIO.h"

#include "MukScene.h"
#include "MukException.h"
#include "MukObstacle.h"
#include "mukIO.h"
#include "MukStringToolkit.h"
#include "PathCollection.h"
#include "version.h"

#include "PlannerFactory.h"
#include "PrunerFactory.h"
#include "InterpolatorFactory.h"
#include "NavigatorFactory.h"

#include "gstd/XmlNode.h"

#include <vtkPolyData.h>

#include <boost/filesystem.hpp>
#include <boost/serialization/vector.hpp>

#include <memory>

namespace
{
  namespace fs = boost::filesystem;
  // searches for node <version> ... </version> and returns the LibVersion (converted from the corresponding string) from it
  gris::muk::LibVersion getLibVersion(const std::vector<gris::XmlNode>& xmlNodes);
}

namespace gris
{
namespace muk
{
  /**
  */
  bool MukSceneLegacyIO::load(const std::string& filename, const XmlNode& root, MukScene* dummyScene)
  {
    auto children = root.getChildren();
    // version 
    auto libVersion = ::getLibVersion(children);

    if (libVersion < LibVersion(0, 4, 1, 0))
    {
      std::stringstream ss;
      ss << "found version: " << libVersion.mMajor << " " << libVersion.mMinor << " " << libVersion.mRevision << " " << libVersion.mBuild;
      throw MUK_EXCEPTION_SIMPLE("Unable to load scene-file. \nThis variant of saving a scene is no longer supported.");
    }
    if (libVersion < LibVersion(0, 4, 2, 0))
    {
      load_v_0_4_1_0(filename, root, dummyScene);
      return true;
    }
    return false;
  }

  /**
  */
  bool MukSceneLegacyIO::load_v_0_4_1_0(const std::string& filename, const XmlNode& root, MukScene* dummyScene)
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
        auto tmp = obj.getChild("Name");
        pObj->setName(tmp.getValue());
        tmp = obj.getChild("File");
        auto file_name = fs::path(dummyScene->mLocalBasePath) / tmp.getValue();
        pObj->setFileName(file_name.generic_string());
        pObj->load();
        tmp = obj.getChild("Active");
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
        auto& coll    = dummyScene->getPathCollection(key);
        {
          if (key=="SSC" || key == "RLinferior")
          {
            coll.addObstacle("schaedelCentered");
            coll.addObstacle("IACCentered");
          }
          else if (key=="Cochlea")
          {
            coll.addObstacle("schaedelCentered");
            coll.addObstacle("cochleaCentered");
          }
        }

        auto pProbDef = coll.getProblemDefinition();
        auto tmp = ndColl.getChild("Radius");
        pProbDef->setRadius(std::atof(tmp.getValue()));
        tmp = ndColl.getChild("SafetyDistance");
        pProbDef->setSafetyDist(std::atof(tmp.getValue()));
        tmp = ndColl.getChild("Kappa");
        pProbDef->setKappa(std::atof(tmp.getValue()));
        tmp = ndColl.getChild("Waypoints");
        {
          auto subNode = tmp.getChild("File");
          Waypoints wp;
          const fs::path fn = fs::path(filename).parent_path() / subNode.getValue();
            
          if (!fs::is_regular_file(fn))
          {
            LOG_LINE << __FUNCTION__ << "   Path: " << key << "   File does not exist: " << fn.string();
            dummyScene->deletePathCollection(key);
            continue;
          }
          std::ifstream ifs(fn.string());
          boost::archive::text_iarchive ia(ifs);
          ia >> wp;

          auto pStart = std::make_unique<MukStateRegion>();
          pStart->setCenter(wp.getStartPoint());
          pStart->setPhi(0);
          pStart->setResolution(0);
          pProbDef->addStartRegion(std::move(pStart)); 

          auto pGoal = std::make_unique<MukStateRegion>();
          pGoal->setCenter(wp.getGoalPoint());
          pGoal->setPhi(0);
          pGoal->setResolution(0);
          pProbDef->addGoalRegion(std::move(pGoal));

          int N = (int)wp.size();
          for (int i(1); i<N-1; ++i)
          {
            auto state = wp.getPoint(i);
            pProbDef->insertWaypoint(state, i-1);
          }
        }
        tmp = ndColl.getChild("MukPaths");
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
        auto props = node.getChildren();
        std::vector<std::string> propNames;
        planner->getPropertyNames(propNames);
        for (const auto& prop : props)
        {
          std::string key = prop.getName();
          if (key == "Type" && std::any_of(propNames.begin(), propNames.end(), [&] (const std::string& str) { return key == str; }))
          {
            auto val = prop.getValue();
            planner->setProperty(key, val);
          }
          if (key == "Bounds-Min" || key == "Bounds-Max")
          {
            auto val = prop.getValue();
            auto collKeys = dummyScene->getPathKeys();
            for (const auto& coll : collKeys)
            {
              dummyScene->getPathCollection(coll).getProblemDefinition()->setProperty(key, val);
            }
          }
        }
        dummyScene->setPlanner(std::move(planner));
      }
    }
    node = configNode.getChild("Planning").getChild("Pruner");
    {
      auto tmp = node.getChild("Type");
      auto pruner = GetPrunerFactory().create(tmp.getValue());
      tmp = node.getChild("MaxStepSize");
      pruner->setMaxStepSize(std::atof(tmp.getValue()));
      tmp = node.getChild("MaxDistance");
      pruner->setMaxDistance(std::atof(tmp.getValue()));
      dummyScene->setPruner(std::move(pruner));
    }
    node = configNode.getChild("Planning").getChild("Interpolator");
    {
      auto tmp = node.getChild("Type");
      auto interpolator = GetInterpolatorFactory().create(tmp.getValue());
      tmp = node.getChild("InterpolationType");
      const char* str = tmp.getValue();
      int idx = IInterpolator::typeFromString(str);
      if (idx==-1)
        interpolator->setInterpolationType(IInterpolator::EnInterpolationTypes(0));
      else
        interpolator->setInterpolationType(IInterpolator::EnInterpolationTypes(idx));
      tmp = node.getChild("PointsPerSegment");
      interpolator->setPointsPerSegment(std::atoi(tmp.getValue()));
      tmp = node.getChild("Resolution");
      interpolator->setResolution(std::atof(tmp.getValue()));
      dummyScene->setInterpolator(std::move(interpolator));
    }
    return true;
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