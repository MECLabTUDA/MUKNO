#include "private/muk.pch"
#include "private/IStateValidityChecker.h"
#include "private/StateSamplerIBounds.h"
#include "private/MukOmplSetup.h"
#include "private/pathplanning_tools.h"
#include "private/types.h"

#include "OmplPlanner.h"

#include "MukCommon/MukException.h"
#include "MukCommon/MukProblemDefinition.h"

#include <ompl/base/ProblemDefinition.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    OmplPlanner::OmplPlanner()
      : IPathPlanner()
      , mpSetup(std::make_unique<MukOmplSetup>())
    {
    }

    /**
    */
    OmplPlanner::~OmplPlanner()
    {
    }


    /**
    */
    void OmplPlanner::initialize()
    {
      if (nullptr == mpCollisionDetector)
      {
        throw MUK_EXCEPTION("Planner cannot be initialized", "CollisionDetector and/or Bounds not yet set");
      }
      mpSetup->initialize(mpCollisionDetector);
    }

    /** \brief Updates the whole internal ompl::ProblemDefinition of a planner

      Sets the bounds, precision parameters, and start and goal states.
      \param switched: Indicates if a planner internally plans from goal to start instead of start to goal. 
                       This is useful for RRTs as in many medical setups it is easier to plan from the dense region inside towards the outside.
    */
    void OmplPlanner::update(bool switched)
    {
      if  (nullptr == mpSetup->mpValidityChecker)
      {
        throw MUK_EXCEPTION("OmplSetup not initialized", "call member function initialize() first");
      }
      auto pProbDef = mpProbDef.lock();
      if (!pProbDef)
      {
        throw MUK_EXCEPTION("No valid ProblemDefinition", "create a path collection first ");
      }

      mpSetup->mpValidityChecker->setBounds(&pProbDef->getBounds());
      const auto* samplingData =  pProbDef->getSamplingData();
      mpSetup->initialize(pProbDef->getBounds()); // this will update the bounds for the factory

      mpSetup->mpValidityChecker->setMaxDistance(pProbDef->getRadius() + pProbDef->getSafetyDist());
      {
        mpSetup->mpProbDef->clearStartStates();
        auto states = switched ? pProbDef->getGoalStates() : pProbDef->getStartStates();
        if (switched)
        {
          for (auto& state : states)
            state.tangent *= -1;
        }
        size_t failCounter(0);
        for (size_t i(0); i<states.size(); ++i)
        {
          auto state = convert(states[i], mpSetup->mpSpace);
          if (!mpSetup->mpValidityChecker->isValid(state.get()))
          {
            ++failCounter;
          }
          else
          {
            mpSetup->mpProbDef->addStartState(state);
          }
        }
        if(failCounter)
        {
          LOG_LINE << "Warning: " << failCounter << " start states do not satisfy the bounds!";
          if (switched)
            LOG_LINE << "   Reminder: Planner internally switched start and goal";
        }
      }
      {
        mpSetup->mpGoal->clear();
        auto states = switched ? pProbDef->getStartStates() : pProbDef->getGoalStates();
        if (switched)
        {
          for (auto& state : states)
            state.tangent *= -1;
        }
        size_t failCounter(0);
        for (size_t i(0); i<states.size(); ++i)
        {
          auto state = convert(states[i], mpSetup->mpSpace);
          if (!mpSetup->mpValidityChecker->isValid(state.get()))
          {
            ++failCounter;
          }
          else
          {
            mpSetup->mpGoal->addState(state);
          }
        }
        if (failCounter)
        {
          LOG_LINE <<  "Warning: " << failCounter << " goal states do not satisfy the bounds!";
          if (switched)
            LOG_LINE << "   Reminder: Planner internally switched start and goal";
        }
        dynamic_cast<ob::MukGoalStates*>(mpSetup->mpProbDef->getGoal().get())->setThreshold(pProbDef->getGoalThreshold());
        dynamic_cast<ob::MukGoalStates*>(mpSetup->mpProbDef->getGoal().get())->setAngleThreshold(pProbDef->getGoalAngleThreshold());
      }
    }

    /**
    */
    void OmplPlanner::clearSearch()
    {
      mpSetup->mpProbDef->clearSolutionPaths();
    }

  }
}