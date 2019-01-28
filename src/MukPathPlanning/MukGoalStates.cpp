#include "private/muk.pch"
#include "private/MukGoalStates.h"
#include "private/types.h"
#include "private/pathplanning_tools.h"

#include "MukCommon/gris_math.h"
#include "MukCommon/geometry.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukVector.h"
#include "MukCommon/gris_math.h"

namespace
{  
  using gris::muk::wideAngle;
  using namespace gris::muk;
  using namespace ompl::geometric;

  inline double euclideanDistance(const MukStateType* st1, const MukStateType* st2)
  {
    return std::sqrt( 
      std::pow(st1->getX()-st2->getX(), 2) 
      + std::pow(st1->getY()-st2->getY(), 2) 
      + std::pow(st1->getZ()-st2->getZ(), 2));
  }

  inline double euclideanDistance(const ob::State* st1, const ob::State* st2)
  {
    const auto* p1 = st1->as<MukStateType>();
    const auto* p2 = st2->as<MukStateType>();
    return euclideanDistance(p1,p2);
  }
}

namespace ompl
{
namespace base
{
  bool MukGoalStates::s_UseOldDirection = true;

  /**
  */
  MukGoalStates::MukGoalStates(const SpaceInformationPtr &si)
    : GoalStates(si)
    , mAngleThreshold(0.125*gris::M_PI)
  {
  }

  /**
  */
  MukGoalStates::~MukGoalStates()
  {
  }
  
  /** \brief Compute the distance to the goal (heuristic). 
  
      This function is the one used in computing the distance to the goal in a call to isSatisfied() 
  */
  double MukGoalStates::distanceGoal(const State *st) const
  {
    double minDist = std::numeric_limits<double>::infinity();
    auto* pState = st->as<MukStateType>();
    for (size_t i(0); i<states_.size(); ++i)
    {
      auto* pGoal = states_[i]->as<MukStateType>();
      double d = euclideanDistance(pState, pGoal);
      // set distance of state inside threshold and exceeding angle to be never inside threshold
      if (d < threshold_)
      {
        const double angle = MukGoalStates::angleDistance(pGoal, st);
        if (angle > mAngleThreshold)
        {
          d += threshold_;
        }
      }      
      if (d < minDist)
        minDist = d;
    }
    return minDist;
  }

  /** \brief computation of the angle between two states
  */
  double MukGoalStates::angleDistance(const State* goalState, const State* state)
  {
    if (s_UseOldDirection)
    {
      const auto* pState = dynamic_cast<const geometric::MukStateType*>(state);
      auto t = gris::Vec3d(pState->rotation().x, pState->rotation().y, pState->rotation().z);
      const auto* pGoal = dynamic_cast<const geometric::MukStateType*>(goalState);
      auto t_goal = gris::Vec3d(pGoal->rotation().x, pGoal->rotation().y, pGoal->rotation().z);
      return wideAngle(t, t_goal);
    }
    else
    {
      using namespace gris::muk;
      using namespace geometric;
      MukState query = fromSE3(*state->as<MukStateType>());
      MukState goal  = fromSE3(*goalState->as<MukStateType>());
      return wideAngle(query.tangent, goal.tangent);
    }
  }

  /**
  */
  double MukGoalStates::euclideanDistance(const State* goalState, const State* state)
  {
    return ::euclideanDistance(goalState, state);
  }

  /** \brief returns only the angle distance of a state to a goal region
  */
  double MukGoalStates::angleDistance(const State* state)
  {
    double dist = std::numeric_limits<double>::infinity();
    size_t idx(0);

    // check, which states are reached in terms of the Euclidean distance
    std::vector<base::State*> insideBall;
    std::copy_if(states_.begin(), states_.end(), back_inserter(insideBall), [&] (const auto* goalState)
    {
      return euclideanDistance(goalState, state) < threshold_;
    });

    if (insideBall.empty())
      return gris::M_PI;

    auto iter = std::min_element(insideBall.begin(), insideBall.end(), [&] (const auto* lhs, const auto* rhs)
      {
        return angleDistance(lhs,state) < angleDistance(rhs,state);
      });
    return angleDistance(*iter, state);
  }
}
}