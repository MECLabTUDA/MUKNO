#pragma once

#include "MukCommon/MukProblemDefinition.h"

namespace gris
{
  namespace muk
  {
    /** \brief old class to compute the goal deviation. 

      should be replaced by new interface of MukPath which already safes this information
    */
    class GoalAngleAnalyzer
    {
      public:
        GoalAngleAnalyzer(const MukProblemDefinition& obj);

      public:
        double euclideanDistance(const MukState& state) const;
        double angleDistance(const MukState& state) const;

      private:
        std::vector<MukState> mGoalStates;
    };

  }
}
