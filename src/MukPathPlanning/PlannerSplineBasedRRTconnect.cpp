#include "private/muk.pch"
#include "private/SplineBasedRRTconnect.h"
#include "private/MukOmplSetup.h"
#include "private/pathplanning_tools.h"
#include "private/types.h"

#include "PlannerSplineBasedRRTconnect.h"

#include "MukCommon/MukException.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/MukProblemDefinition.h"

namespace
{
  using namespace gris::muk;
}

namespace gris
{
  namespace muk
  {
    REGISTER_PLANNER(PlannerSplineBasedRRTconnect);

    /**
    */
    PlannerSplineBasedRRTconnect::PlannerSplineBasedRRTconnect()
      : OmplPlanner()
      , mGoalBias(0.05)
      , mNearestBallRadius(1.0)
      , mConnectionRange(2.0)
    { 
      declareProperty<double>("GoalBias", [&] (double d) { setGoalBias(d); }, [&] () { return getGoalBias(); });
      declareProperty<double>("NearestBallRadius", [&] (double d) { setNearestBallRadius(d); }, [&] () { return getNearestBallRadius(); });
      declareProperty<double>("Connection_Range", [&] (double d) { setConnectionRange(d); }, [&] () { return getConnectionRange(); });
    }

    /**
    */
    PlannerSplineBasedRRTconnect::~PlannerSplineBasedRRTconnect()
    {
    }

    /** \brief Initialized the OMPL configuration

    Creates the State space (SE3) and SpaceInformation.
    Sets the ompl-State-Sampler-Allocator, the ompl-StateValidityChecker and the ompl-ProblemDefinition

    defines additional parameteres that are not yet accessible from outside. (GoalBias, GoalProximity)
    */
    void PlannerSplineBasedRRTconnect::initialize()
    {
      OmplPlanner::initialize();
      mpPlanner = std::make_unique<og::SplineBasedRRTconnect>(mpSetup->mpSpaceInfo);
      mpPlanner->setProblemDefinition(mpSetup->mpProbDef);
    }

    /** \brief computes a solution path
    */
    bool PlannerSplineBasedRRTconnect::calculate()
    {
      ob::PlannerStatus solved = mpPlanner->solve(mCalculationTime);

      if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
        LOG_LINE << "PlannerSplineBasedRRTconnect: Solution Found!";
      else if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
        LOG_LINE << "PlannerSplineBasedRRTconnect: Approximate Solution Found!";
      else
        LOG_LINE << "PlannerSplineBasedRRTconnect: Could not find Solution";
      return solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION;
    }

    /**
    */
    void PlannerSplineBasedRRTconnect::update()
    {
      if (nullptr == mpPlanner.get())
      {
        throw MUK_EXCEPTION("Planner not initialized", "call member function initialize() first");
      }
      OmplPlanner::update();
      ob::MukGoalStates::s_UseOldDirection = true;
      auto pProbDef = mpProbDef.lock();
      if (pProbDef)
      {
        mpPlanner->setKappa(pProbDef->getKappa());
      }
      mpPlanner->setStepSize(mStepSize);
      mpPlanner->setGoalBias(mGoalBias);
      mpPlanner->setNearestBallRadius(mNearestBallRadius);
      mpPlanner->setConnectionRange(mConnectionRange);
    }

    /**
    */
    void PlannerSplineBasedRRTconnect::clearSearch()
    {
      OmplPlanner::clearSearch();
      mpPlanner->restartSearch(); 
    }

    /** \brief Creates a MukPath from the computed internal ompl-SolutionPath
    */
    MukPath PlannerSplineBasedRRTconnect::extractPath(size_t idx) const
    {
      auto pOmplPath = mpPlanner->getProblemDefinition()->getSolutionPath();
      if (nullptr == pOmplPath.get())
      {
        return MukPath();
      }
      MukPath path;
      std::vector<ob::State*> omplStates = pOmplPath->as<og::PathGeometric>()->getStates();
      {
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
      return path;
    }

    /**
    // atm: only one starting vertex supported
    */
    MukPathGraph PlannerSplineBasedRRTconnect::extractRoadmap() const
    {
      if (nullptr != mpPlanner)
      {
        ob::PlannerData plannerData (mpPlanner->getSpaceInformation());
        mpPlanner->getPlannerData(plannerData);
        auto graph = gris::muk::extractRoadmap(plannerData);
        graph.setSizeStart(mpPlanner->getNumberOfStartTreeRoots());
        return graph;
      }
      else
        return MukPathGraph();
    }

    size_t PlannerSplineBasedRRTconnect::availablePaths() const
    {
      return mpPlanner->getProblemDefinition()->getSolutionCount();
    }
  }
}