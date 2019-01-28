#include "private/muk.pch"
#include "private/GoalAnalyzer.h"

#include "MukCommon/geometry.h"

namespace gris
{
namespace muk
{

  /**
  */
  GoalAngleAnalyzer::GoalAngleAnalyzer(const MukProblemDefinition& obj)
  {
    mGoalStates = obj.getGoalStates();
  }

  /**
  */
  double GoalAngleAnalyzer::euclideanDistance(const MukState& state) const
  {
    double minDist = std::numeric_limits<double>::infinity();
    for (size_t i(0); i<mGoalStates.size(); ++i)
    {
      double d = (mGoalStates[i].coords - state.coords).squaredNorm();
      if (d < minDist)
        minDist = d;
    }
    return std::sqrt(minDist);
  }

  /**
  */
  double GoalAngleAnalyzer::angleDistance(const MukState& state) const
  {
    if (mGoalStates.empty())
      return std::numeric_limits<double>::quiet_NaN();
    double minDist = std::numeric_limits<double>::infinity();
    size_t idx = 0;
    for (size_t i(0); i<mGoalStates.size(); ++i)
    {
      double d = (mGoalStates[i].coords - state.coords).squaredNorm();
      if (d < minDist)
      {
        minDist = d;
        idx = i;
      }
    }
    return wideAngle(mGoalStates[idx].tangent, state.tangent);
  }
}
}
