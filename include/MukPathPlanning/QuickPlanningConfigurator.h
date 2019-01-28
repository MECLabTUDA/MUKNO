#pragma once

#include "muk_pathplanning_api.h"

#include <map>

namespace gris
{
  namespace muk
  {
    /** \brief A class that provides several important default parameter
    */
    class MUK_PP_API QuickPlanningConfigurator
    {
      public:
        static QuickPlanningConfigurator create(const std::string& plannerName);

      public:
        const std::string& getPlanner()      const { return mPlanner; }
        const std::string& getPruner()       const { return mPruner; }
        const std::string& getInterpolator() const { return mInterpolator; }
        size_t getSphereResolution()  const { return mSphereResolution; }
        size_t getNewPaths()          const { return mNewPaths; }
        double getCalcTime()          const { return mCalcTime; }
        double getStepSize()          const { return mStepSize; }

      private:
        std::string mPlanner;
        std::string mPruner;
        std::string mInterpolator;
        size_t mSphereResolution; // resolution of direction samping, differs between linear and nonlinear
        size_t mNewPaths;
        double mCalcTime;
        double mStepSize;
    };



  }
}
