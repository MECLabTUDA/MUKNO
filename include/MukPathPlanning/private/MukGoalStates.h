#pragma once

#include <ompl/base/goals/GoalStates.h>

namespace ompl
{
  namespace base
  {

    class MukGoalStates : public GoalStates
    {
      public:
        MukGoalStates(const SpaceInformationPtr &si);
        virtual ~MukGoalStates();

      public:
        static double angleDistance(const State* goalState, const State* state);
        static double euclideanDistance(const State* goalState, const State* state);
        
      public:
        virtual double distanceGoal(const State *st) const;
        double angleDistance(const State* state);

        void   setAngleThreshold(double d)      { mAngleThreshold = d; }
        double getAngleThreshold()        const { return mAngleThreshold; }

        static bool s_UseOldDirection;

      private:
        double mAngleThreshold;
    };
    
  }
}
