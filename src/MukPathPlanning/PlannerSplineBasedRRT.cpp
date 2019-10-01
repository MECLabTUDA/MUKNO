#include "private/muk.pch"
#include "private/SplineBasedRRT.h"
#include "private/MukOmplSetup.h"
#include "private/pathplanning_tools.h"
#include "private/types.h"

#include "PlannerSplineBasedRRT.h"

#include "MukCommon/MukException.h"
#include "MukCommon/MukProblemDefinition.h"
#include "MukCommon/PlannerFactory.h"

namespace
{
  using namespace gris::muk;
}

namespace gris
{
  namespace muk
  {
    REGISTER_PLANNER(PlannerSplineBasedRRT);
            
    /**
    */
    PlannerSplineBasedRRT::PlannerSplineBasedRRT()
      : OmplPlanner()
      , mGoalBias(0.05)
      , mNearestBallRadius(1.0)
      , mSwitchStartAndGoal(true)
    { 
      declareProperty<double>("GoalBias", MUK_SET(double, setGoalBias), MUK_GET(getGoalBias));
      declareProperty<double>("NearestBallRadius", MUK_SET(double, setNearestBallRadius), MUK_GET(getNearestBallRadius));
      declareProperty<bool>("SwitchStartAndGoal",  MUK_SET(bool, setSwitchStartAndGoal), MUK_GET(getSwitchStartAndGoal));
    }
    
    /**
    */
    PlannerSplineBasedRRT::~PlannerSplineBasedRRT()
    {
    }

    /**
    */
    void PlannerSplineBasedRRT::setSwitchStartAndGoal(bool d)
    { 
      mSwitchStartAndGoal = d;
      if (nullptr != mpPlanner.get()) // bugfix, if no check, it will throw when a scene is loaded
        update();
    }

    /** \brief Initialized the OMPL configuration

      Creates the State space (SE3) and SpaceInformation.
      Sets the ompl-State-Sampler-Allocator, the ompl-StateValidityChecker and the ompl-ProblemDefinition

      defines additional parameteres that are not yet accessible from outside. (GoalBias, GoalProximity)
    */
    void PlannerSplineBasedRRT::initialize()
    {
      OmplPlanner::initialize();
      mpPlanner = std::make_unique<og::SplineBasedRRT>(mpSetup->mpSpaceInfo);
      mpPlanner->setProblemDefinition(mpSetup->mpProbDef);
    }
    
    /** \brief computes a solution path
    */
    bool PlannerSplineBasedRRT::calculate()
    {
      ob::PlannerStatus solved = mpPlanner->solve(mCalculationTime);
      if (mVerbose)
      {
        if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
          LOG_LINE << "  PlannerSplineBasedRRT: Solution Found!";
        else if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
          LOG_LINE << "  PlannerSplineBasedRRT: Approximate Solution Found!";
        else
          LOG_LINE << "  PlannerSplineBasedRRT: Could not find Solution";
      }
      return solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION;
    }

    /**
    */
    void PlannerSplineBasedRRT::update()
    {
      if (nullptr == mpPlanner.get())
      {
        throw MUK_EXCEPTION("Planner not initialized", "call member function initialize() first");
      }
      OmplPlanner::update(mSwitchStartAndGoal);
      ob::MukGoalStates::s_UseOldDirection = true;
      auto pProbDef = mpProbDef.lock();
      if (pProbDef)
      {
        mpPlanner->setKappa(pProbDef->getKappa());
      }
      mpPlanner->setVerbose(mVerbose);
      mpPlanner->setGoalBias(mGoalBias);
      mpPlanner->setStepSize(mStepSize);      
      mpPlanner->setNearestBallRadius(mNearestBallRadius);
    }

    /**
    */
    void PlannerSplineBasedRRT::clearSearch()
    {
      OmplPlanner::clearSearch();
      mpPlanner->restartSearch();
    }
    
    /** \brief Creates a MukPath from the computed internal ompl-SolutionPath
    */
    MukPath PlannerSplineBasedRRT::extractPath(size_t idx) const
    {
      auto pOmplPath = mpPlanner->getProblemDefinition()->getSolutionPath();
      MukPath path;
      {
        std::vector<ob::State*> omplStates = pOmplPath->as<og::PathGeometric>()->getStates();
        const size_t N = omplStates.size();
        std::vector<MukState> states;
        states.reserve(N);
        for (auto* pState : omplStates)
        {
          auto* v = pState->as<og::MukStateType>();
          const auto& rot = v->rotation();
          MukState m(Vec3d(v->getX(), v->getY(), v->getZ()), Vec3d(rot.x, rot.y, rot.z));
          states.push_back(m);
        }
        path.setStates(states);
      }
      auto pProbDef = mpProbDef.lock();
      if (pProbDef)
      {
        path.setRadius(pProbDef->getRadius());
      }
      const auto& stats = mpPlanner->getStats();
      path.setGoalAngle(stats.goalAngle);
      path.setGoalDist(stats.goalDist);
      path.setNumberOfSearchStates(stats.numberOfStatesCreated);
      path.setTimeSpend(stats.millisecondsSpend);
      path.setApproximated(stats.success);
      // rearrange the path
      if (mSwitchStartAndGoal)
        path = swapDirection(path);
      return path;
    }

    /**
      // atm: only one starting vertex supported
    */
    MukPathGraph PlannerSplineBasedRRT::extractRoadmap() const
    {
      if (nullptr != mpPlanner)
      {
        ob::PlannerData plannerData (mpPlanner->getSpaceInformation());
        mpPlanner->getPlannerData(plannerData);
        auto graph = gris::muk::extractRoadmap(plannerData);
        graph.setSizeStart(plannerData.numStartVertices());
        return graph;
      }
      else 
      {
        return MukPathGraph();
      }
    }

    size_t PlannerSplineBasedRRT::availablePaths() const
    {
      return mpPlanner->getProblemDefinition()->getSolutionCount();
    }
  }
}