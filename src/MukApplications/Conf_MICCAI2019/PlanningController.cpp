#include "private/muk.pch"
#include "PlanningController.h"

#include "MukCommon/MukException.h"
#include "MukCommon/MukIO.h"
#include "MukCommon/IPathPlanner.h"
#include "MukCommon/IPathOptimizer.h"
#include "MukCommon/IInterpolator.h"
#include "MukCommon/PathCollection.h"

#include "MukAppModels/PlanningModel.h"

#include "gstd/XmlDocument.h"
#include "gstd/XmlNode.h"

#include <boost/filesystem.hpp>

#include <chrono>

namespace
{
  const std::string DummyOptimizer = "PrunerDummy";
}

namespace gris
{
namespace muk
{
  std::vector<long long> PlanningController::milliseconds = {};

  /**
  */
  PlanningController::PlanningController()
    : mPlanningTime(1.0)
  {
  }
  
  /**
  */
  void PlanningController::plan(const std::string& key)
  {
    // set to RRT
    {
      mpPlan->setPlanner(mPlanner);
      auto* planner = mpScene->getPlanner();
      for (const auto& pair : mParamsBezierPlanner)
        planner->setProperty(pair.first, pair.second);
      mpPlan->setPruner(DummyOptimizer);
      /*auto optimizer = mpScene->getPruner();
      for (const auto& pair : mParamsBezierOptimizer)
      {
        if (optimizer->hasProperty(pair.first))
          optimizer->setProperty(pair.first, pair.second);
      }*/
    }
    mpPlan->setActivePathCollection(key);
    mpPlan->setInterpolator(mInterpolator);
    mpPlan->configurePlanning(key);
    try
    {
      mpPlan->createPaths(key, mPlanningTime);
    }
    catch(std::exception& e)
    {
      LOG_LINE << "   !!! Exception during planning: " << e.what();
    }
  }

  /**
  */
  void PlanningController::optimize(const std::string& reference, const std::string& target)
  {
    // set to SCO
    mpPlan->setPruner(mOptimizer);
    // configure planning
    const auto& collRef  = mpScene->getPathCollection(reference);
    const auto& pathsRef = collRef.getPaths();
    mpPlan->setActivePathCollection(target);
    mpPlan->configurePlanning(target);
    // optimize each path
    auto& collTar = mpScene->getPathCollection(target);
    for (size_t i(0); i<pathsRef.size(); ++i)
    {
      try
      {
        //collTar.insertPath(pathsRef[i]);
        mpPlan->setActivePathIdx(static_cast<int>(collTar.getPaths().size()) - 1);
        const auto start = std::chrono::high_resolution_clock::now();
        auto optimizer = mpScene->getPruner();
        for (const auto& pair : mParamsBezierOptimizer1)
          optimizer->setProperty(pair.first, pair.second);
        //mpPlan->updatePruner();
        auto path = optimizer->calculate(pathsRef[i]);
        for (const auto& pair : mParamsBezierOptimizer2)
          optimizer->setProperty(pair.first, pair.second);
        path = optimizer->calculate(path);
        const auto end = std::chrono::high_resolution_clock::now();
        const auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        LOG_LINE << "add time: " << time;
        milliseconds.push_back(time);
        if (optimizer->valid())
          collTar.insertPath(path);
      }
      catch(std::exception& e)
      {
        LOG_LINE << "   !!! Exception during optimization: " << e.what();
      }
    }
  }

  /**
  */
  void PlanningController::save(const std::string& outputDir, const std::string& key)
  {
    const auto& paths   = mpScene->getPathCollection(key).getPaths();
    if (paths.empty())
      return;
    namespace fs        = boost::filesystem;
    const auto dir      = fs::path(outputDir);
    for (size_t i(0); i<paths.size(); ++i)
    {
      const auto fn = dir / ( key + "_" + std::to_string(i) + ".txt" );
      saveMukPathAsTxt(paths[i], fn.string().c_str());
    }
  }

  // ------------------------------------------------------------------------------------
  // static functions
  // ------------------------------------------------------------------------------------

  /**
  */
  void PlanningController::readConfigFile(const std::string& configFile, PlanningController& obj)
  {
    namespace fs = boost::filesystem;
    if (!fs::is_regular_file(configFile))
    {
      throw MUK_EXCEPTION("config file does not exist", configFile.c_str());
    }
    auto pDoc = XmlDocument::read(configFile.c_str());
    const auto rootNode = pDoc->getRoot();
    //
    auto children = rootNode.getChild("Accesses").getChildren();
    for (const auto& node : children)
      obj.mAccesses.push_back(node.getValue());
    //
    obj.mPlanner      = rootNode.getChild("Planner").getValue();
    obj.mOptimizer    = rootNode.getChild("Optimizer").getValue();
    obj.mInterpolator = rootNode.getChild("Interpolator").getValue();
    //
    auto node  = rootNode.getChild("PlannerParameters");
    auto nodes = node.getChildren();
    for (const auto& node : nodes)
    {
      auto key = std::string(node.getAttribute("planner").getValue());
      auto ndParams = node.getChildren();
      if (key == obj.mPlanner)
      {
        for(const auto& node : ndParams)
          obj.mParamsBezierPlanner.push_back(std::make_pair(node.getName(), node.getValue()));
      }
    }
    node  = rootNode.getChild("OptimizerParameters");
    nodes = node.getChildren();
    if (nodes.size() != 2)
      throw MUK_EXCEPTION_SIMPLE("expecting two parameter sets for optimizer");
    for (size_t i(0); i<2; ++i)
    {
      const auto& node = nodes[i];
      auto key = std::string(node.getAttribute("optimizer").getValue());
      auto ndParams = node.getChildren();
      if (key == obj.mOptimizer)
      {
        for(const auto& node : ndParams)
        {
          if (i==0)
            obj.mParamsBezierOptimizer1.push_back(std::make_pair(node.getName(), node.getValue()));
          else
            obj.mParamsBezierOptimizer2.push_back(std::make_pair(node.getName(), node.getValue()));
        }
      }
    }
    node  = rootNode.getChild("InterpolatorParameters");
    nodes = node.getChildren();
    for (const auto& node : nodes)
    {
      auto key = std::string(node.getAttribute("interpolator").getValue());
      auto ndParams = node.getChildren();
      if (key == obj.mInterpolator)
      {
        for(const auto& node : ndParams)
          obj.mParamsBezierInterpolator.push_back(std::make_pair(node.getName(), node.getValue()));
      }
    }
  }
}
}