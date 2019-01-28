#include "private/muk.pch"
#include "ApplicationModel.h"
#include "PlanningModel.h"
#include "VisualizationModel.h"
#include "ProblemDefinitionModel.h"

#include "MukCommon/MukException.h"
#include "MukCommon/mukIO.h"
#include "MukCommon/MukStringToolkit.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/PrunerFactory.h"
#include "MukCommon/InterpolatorFactory.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/MukPathGraph.h"

#include "MukEvaluation/statistics.h"

#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisObstacle.h"

#include "gstd/XmlDocument.h"
#include "gstd/XmlNode.h"

#include "boost/filesystem.hpp"
#include <boost/format.hpp>

#include <chrono>

namespace
{
  namespace fs = boost::filesystem;
  using gris::Vec3d;
  using namespace gris::muk;
  Vec3d fromString(const char*);
}

namespace gris
{
namespace muk
{
  /**
  */
  PlanningModel::PlanningModel(std::shared_ptr<MukScene> pScene)
    : BaseModel()
    , mpScene(pScene)
    , mActivePathCollection("")
    , mActivePathIdx(Invalid_Index)
  {
  }

  /**
  */
  PlanningModel::~PlanningModel()
  {
  }
  
  /** \brief Creates a planner and passes it to MukScene
  */
  void PlanningModel::setPlanner(const std::string& name)
  { 
    LOG_LINE << "PlanningModel: change planner to " << name;
    std::unique_ptr<IPathPlanner> newPlanner = GetPlannerFactory().create(name);
    newPlanner->clone(mpScene->getPlanner());
    mpScene->setPlanner(std::move(newPlanner));
    mpScene->getPlanner()->initialize();
  }

  /** \brief Creates a pruner and passes it to MukScene
  */
  void PlanningModel::setPruner(const std::string& name)
  {
    std::unique_ptr<IPathPruner> newPruner = GetPrunerFactory().create(name);
    newPruner->clone(mpScene->getPruner());
    mpScene->setPruner(std::move(newPruner));
  }

  /** \brief Creates an interpolator and passes it to MukScene
  */
  void PlanningModel::setInterpolator(const std::string& name)
  {
    std::unique_ptr<IInterpolator> newInterp = GetInterpolatorFactory().create(name);
    newInterp->clone(mpScene->getInterpolator());
    mpScene->setInterpolator(std::move(newInterp));
  }

  /** \brief Tries to create a specific number of paths for a given PathCollection 
  */
  void PlanningModel::createPaths(const std::string& name, size_t numNewPaths)
  {
    auto& coll = mpScene->getPathCollection(name);
    auto* pPlanner       = mpScene->getPlanner();
    auto* pPruner        = mpScene->getPruner();
    // quick fix for introduction of new Linear Planner, have to change the procedure from numNewPaths to max number of calls
    size_t count(0);
    for (size_t i(0); i<numNewPaths && count <= numNewPaths; ++i)
    {
      pPlanner->clearSearch();
      bool succeeded = pPlanner->calculate();
      if (succeeded)
      {
        const auto N = pPlanner->availablePaths();
        count += N;
        for (size_t i(0); i<N; ++i)
        {
          auto path = pPlanner->extractPath(i);
          auto prunedPath = pPruner->calculate(path);
          coll.insertPath(prunedPath);
        }
      }
    }
  }

  /** \brief Reconfigures the Planning Model and tries to computes as many paths as possible in the available time

  lets the model compute paths for a certain amount of time 
  adds visualizations of the new paths
  logs how many new paths were actually created in the available time

  \param pathCollection name of the path collection which gets configures
  \param timeAvailable  time available (in seconds) for reapeted calls to solve the motion planning problem
  */
  void PlanningModel::createPaths(const std::string& name, double timeAvailable)
  {
    auto& coll = mpScene->getPathCollection(name);
    auto* pPlanner       = mpScene->getPlanner();
    auto* pPruner        = mpScene->getPruner();
    using namespace std::chrono;
    auto maxTime = 1000*timeAvailable; // milliseconds cast
    long timeSpend(0);
    while(maxTime > timeSpend)
    {
      pPlanner->clearSearch();
      auto start   = high_resolution_clock::now();
      bool succeeded = pPlanner->calculate();
      timeSpend += std::max(1ll,duration_cast<milliseconds>(high_resolution_clock::now() - start).count());
      if (succeeded)
      {
        const auto N = pPlanner->availablePaths();
        for (size_t i(0); i<N; ++i)
        {
          auto path = pPlanner->extractPath(i);
          auto prunedPath = pPruner->calculate(path);
          coll.insertPath(prunedPath);
        }
      }
    }
  }

  /** \brief Tries to update a specific path of a given PathCollection 
  */
  void PlanningModel::updatePath(const std::string& name, int pathIdx)
  {
    auto& coll = mpScene->getPathCollection(name);
    const size_t N = coll.getPaths().size();
    if (pathIdx < 0)
    {
      throw MUK_EXCEPTION((boost::format("unable to update path No.%d") % (pathIdx + 1)).str().c_str(), (boost::format("This is not a valid index!")).str().c_str());
    }
    if (pathIdx >= static_cast<int>(N))
    {
      throw MUK_EXCEPTION((boost::format("unable to update path No.%d") % (pathIdx + 1)).str().c_str(), (boost::format("There are only %d paths in the PathCollection") % N).str().c_str());
    }

    configurePlanning(name);

    auto* pPlanner       = mpScene->getPlanner();
    auto* pPruner        = mpScene->getPruner();
    pPlanner->calculate();
    {
      auto path = pPlanner->extractPath();
      if (!path.isApproximated())
      {
        auto prunedPath = pPruner->calculate(path);
        coll.setPath(pathIdx, prunedPath);
      }
      else
      {
        coll.setPath(pathIdx, MukPath());
      }
    }
  }

  void PlanningModel::loadPath(const std::string& pathCollection, const std::string& filename)
  {
    auto& coll = mpScene->getPathCollection(pathCollection);
    auto path = loadMukPath(filename);
    coll.insertPath(path);
  }

  /** \brief deletes all paths from a pathCollection
  */
  void PlanningModel::clearPaths(const std::string& pathCollectionName)
  {
    mpScene->getPathCollection(pathCollectionName).getPaths().clear();
  }

  /** \brief Older convenience function. 
  
    Used only the planner to update the "active path" (from ApplicationModel) of the "active PathCollection" (from MukScene)
  */
  void PlanningModel::updatePlanner()
  {
    const auto& key = mActivePathCollection;
    if (key.empty())
      return;
    auto& coll = mpScene->getPathCollection(key);

    const int N       = coll.getPaths().size();;
    if (mActivePathIdx >= N)
    {
      throw MUK_EXCEPTION((boost::format("unable to update path No.%d") % (mActivePathIdx + 1)).str().c_str(), (boost::format("There are only %d paths in the PathCollection") % N).str().c_str());
    }
    if (mActivePathIdx <= Invalid_Index)
    {
      throw MUK_EXCEPTION((boost::format("unable to update path No.%d") % (mActivePathIdx + 1)).str().c_str(), (boost::format("This is not a valid index!")).str().c_str());
    }
    
    configurePlanning(key);
    auto* pPlanner = mpScene->getPlanner();
    pPlanner->calculate();
    if (pPlanner->availablePaths() > 0)
    {
      coll.setPath(mActivePathIdx, pPlanner->extractPath());
    }
  }

  /** \brief Older convenience function.

    Used only the pruner to update the "active path" (from ApplicationModel) of the "active PathCollection" (from MukScene)
  */
  void PlanningModel::updatePruner()
  {
    const auto& key = mActivePathCollection;
    if (key.empty())
      return;
    auto& coll = mpScene->getPathCollection(key);

    const int N = coll.getPaths().size();;
    if (mActivePathIdx >= N)
    {
      throw MUK_EXCEPTION((boost::format("unable to update path No.%d") % (mActivePathIdx + 1)).str().c_str(), (boost::format("There are only %d paths in the PathCollection") % N).str().c_str());
    }
    if (mActivePathIdx <= Invalid_Index)
    {
      throw MUK_EXCEPTION((boost::format("unable to update path No.%d") % (mActivePathIdx + 1)).str().c_str(), (boost::format("This is not a valid index!")).str().c_str());
    }

    configurePlanning(key);
    auto* pPruner = mpScene->getPruner();
    auto path     = pPruner->calculate(coll.getPaths()[mActivePathIdx]);
    coll.setPath(mActivePathIdx, path);
  }

  /** \brief Older convenience function.

    Used only the interpolator to update the "active path" (from ApplicationModel) of the "active PathCollection" (from MukScene)
  */
  void PlanningModel::updateInterpolator(int index, MukPath& path)
  {
    const auto& key = mActivePathCollection;
    if (key.empty())
      return;
    auto& coll = mpScene->getPathCollection(key);
    const int N       = coll.getPaths().size();;
    if (index >= N)
    {
      throw MUK_EXCEPTION((boost::format("unable to update path No.%d") % (mActivePathIdx + 1)).str().c_str(), (boost::format("There are only %d paths in the PathCollection") % N).str().c_str());
    }
    if (index <= Invalid_Index)
    {
      throw MUK_EXCEPTION((boost::format("unable to update path No.%d") % (mActivePathIdx + 1)).str().c_str(), (boost::format("This is not a valid index!")).str().c_str());
    }
    
    auto* interpolator  = mpScene->getInterpolator();
    interpolator->setInput(coll.getPaths()[index]);
    interpolator->interpolate();
    path = interpolator->getInterpolation();
  }

  /** \brief Replaces the start and goal states of the PathCollection with those in the visualization
  */
  void PlanningModel::updateProblemDefinition(const std::string& name)
  {
    auto& coll = mpScene->getPathCollection(name);
    clearPaths(name);
    auto& pCollVis = mpModels->pVisModel->getVisScene()->getPathCollection(name);
    auto pProbDef = coll.getProblemDefinition();
    pProbDef->clearStart();
    for (size_t i(0); i<pCollVis->sizeStart(); ++i)
    {
      pProbDef->addStartRegion(pCollVis->getStartRegion(i).asStateRegion(true));
    }
    pProbDef->clearGoal();
    for (size_t i(0); i<pCollVis->sizeGoal(); ++i)
    {
      pProbDef->addGoalRegion(pCollVis->getGoalRegion(i).asStateRegion(false));
    }
  }
  
  /**
  */
  void PlanningModel::insertPathCollection(const std::string& name)
  {
    if (mpScene->hasPathKey(name))
      throw MUK_EXCEPTION("Key already exists", name.c_str());
    mpScene->insertPathCollection(name);
  }
  
  /**
  */
  void PlanningModel::deletePathCollection(const std::string& name)
  {
    bool change = name == mActivePathCollection;
    mpScene->deletePathCollection(name);
    if (change)
    {
      auto keys = mpScene->getPathKeys();
      std::string newActivePath = keys.empty()  ?   "" : keys.back();
      setActivePathCollection(newActivePath);
    }
  }

  /**
  */
  void PlanningModel::setActivePathCollection(const std::string& name)
  {
    mActivePathCollection = name;
    const int idx = name.empty() || mpScene->getPathCollection(name).getPaths().empty()  ?   Invalid_Index : 0;
    setActivePathIdx(idx);
  }
  
  /**
  */
  std::unique_ptr<MukPathGraph> PlanningModel::extractPlanningGraph() const
  {
    auto pObj = std::make_unique<MukPathGraph>();
    *pObj= mpScene->getPlanner()->extractRoadmap();
    return pObj;
  }

  /** \brief Sets all necessary parameters for planner, pruner and interpolator.
  */
  void PlanningModel::configurePlanning(const std::string& name)
  {
    const auto& deactivated = mpScene->getPathCollection(name).getObstacles();
    auto        pDet        = mpScene->getCollisionDetector();
    const auto  existing    = pDet->getKeys();
    for (const auto& key : existing)
    {
      auto pObs = mpScene->getObstacle(key);
      bool active = std::none_of(deactivated.begin(), deactivated.end(), [&] (const auto& str) { return str == key; });
      pObs->setActive(active);
      pDet->setActive(key, active);
    }

    if (mpScene->getCollisionDetector()->isOutOfDate())
    {
      mpScene->getCollisionDetector()->rebuild();
    }

    auto& coll = mpScene->getPathCollection(name);
    auto* pPlanner = mpScene->getPlanner();
    pPlanner->setProblemDefinition(coll.getProblemDefinition());
    pPlanner->update();
    
    auto* pruner = mpScene->getPruner();        
    pruner->setKappa(coll.getProblemDefinition()->getKappa());
    pruner->setMaxDistance(coll.getProblemDefinition()->getRadius() + coll.getProblemDefinition()->getSafetyDist()); 

    auto* interpolator  = mpScene->getInterpolator();
    interpolator->setKappa(coll.getProblemDefinition()->getKappa());
  }

  /**
  */
  void PlanningModel::setSamplingType(const StateSamplerData& data)
  {
    if (mActivePathCollection.empty())
      throw MUK_EXCEPTION_SIMPLE("no path collection available");

    auto pData = std::make_unique<StateSamplerData>(data);
    mpScene->getPathCollection(mActivePathCollection).getProblemDefinition()->setSamplingData(std::move(pData));
  }
}
}
