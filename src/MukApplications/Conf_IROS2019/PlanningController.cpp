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

namespace gris
{
namespace muk
{
  /**
  */
  const std::string PlanningController::Key_Origin_Otobasis = "RL-Access";
  const std::string PlanningController::Key_A_Otobasis      = "RL-Access-A";
  const std::string PlanningController::Key_B_Otobasis      = "RL-Access-B";
  const std::string PlanningController::Key_Origin_SegThor  = "Heart-Access";
  const std::string PlanningController::Key_A_SegThor       = "Heart-Access-A";
  const std::string PlanningController::Key_B_SegThor       = "Heart-Access-B";
  const std::string PlanningController::Planner_A           = "Bevel-Tip-RRT-Connect";
  const std::string PlanningController::Planner_B           = "Spline-Based-RRT-Connect";
  const std::string PlanningController::Optimizer_A         = "PrunerDummy";
  const std::string PlanningController::Optimizer_B         = "BezierToCircularArcs";
  const std::string PlanningController::Interpolator_A      = "Linear-Interpolator";
  const std::string PlanningController::Interpolator_B      = "BezierToCircularArcs-I";
    

  /**
  */
  PlanningController::PlanningController()
    : mPlanningTime(1.0)
  {
  }

  /**
  */
  void PlanningController::setToBezierSplines()
  {
    mpPlan->setPlanner(Planner_B);
    auto* planner = mpScene->getPlanner();
    for (const auto& pair : mParamsBezierPlanner)
      planner->setProperty(pair.first, pair.second);
    mpPlan->setPruner(Optimizer_B);
    auto optimizer = mpScene->getPruner();
    for (const auto& pair : mParamsBezierOptimizer)
      optimizer->setProperty(pair.first, pair.second);
  }

  /**
  */
  void PlanningController::setToCircularArcs()
  {
    mpPlan->setPlanner(Planner_A);
    auto* planner = mpScene->getPlanner();
    for (const auto& pair : mParamsBevelPlanner)
      planner->setProperty(pair.first, pair.second);
    mpPlan->setPruner(Optimizer_A);
    auto optimizer = mpScene->getPruner();
    for (const auto& pair : mParamsBevelOptimizer)
      optimizer->setProperty(pair.first, pair.second);
  }

  /**
  */
  void PlanningController::plan()
  {
    mpPlan->setActivePathCollection(mCurrentKey);
    mpPlan->configurePlanning(mCurrentKey);
    try
    {
      mpPlan->createPaths(mCurrentKey, mPlanningTime);
    }
    catch(std::exception& e)
    {
      LOG_LINE << "   !!! Exception during planning: " << e.what();
    }
  }

  /**
  */
  void PlanningController::save(const std::string& outputDir)
  {
    const auto key    = std::string(mpScene->getPlanner()->name());
    const auto& paths = mpScene->getPathCollection(mCurrentKey).getPaths();
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
    auto node  = rootNode.getChild("PlannerParameters");
    auto nodes = node.getChildren();
    for (const auto& node : nodes)
    {
      auto key = std::string(node.getAttribute("planner").getValue());
      auto ndParams = node.getChildren();
      if (key == Planner_B)
      {
        for(const auto& node : ndParams)
          obj.mParamsBezierPlanner.push_back(std::make_pair(node.getName(), node.getValue()));
      }
      else
      {
        for(const auto& node : ndParams)
          obj.mParamsBevelPlanner.push_back(std::make_pair(node.getName(), node.getValue()));
      }
    }
    node  = rootNode.getChild("OptimizerParameters");
    nodes = node.getChildren();
    for (const auto& node : nodes)
    {
      auto key = std::string(node.getAttribute("optimizer").getValue());
      auto ndParams = node.getChildren();
      if (key == Optimizer_B)
      {
        for(const auto& node : ndParams)
          obj.mParamsBezierOptimizer.push_back(std::make_pair(node.getName(), node.getValue()));
      }
      else
      {
        for(const auto& node : ndParams)
          obj.mParamsBevelOptimizer.push_back(std::make_pair(node.getName(), node.getValue()));
      }
    }
    node  = rootNode.getChild("InterpolatorParameters");
    nodes = node.getChildren();
    for (const auto& node : nodes)
    {
      auto key = std::string(node.getAttribute("interpolator").getValue());
      auto ndParams = node.getChildren();
      if (key == Interpolator_B)
      {
        for(const auto& node : ndParams)
          obj.mParamsBezierInterpolator.push_back(std::make_pair(node.getName(), node.getValue()));
      }
      else
      {
        for(const auto& node : ndParams)
          obj.mParamsBevelInterpolator.push_back(std::make_pair(node.getName(), node.getValue()));
      }
    }
  }
}
}