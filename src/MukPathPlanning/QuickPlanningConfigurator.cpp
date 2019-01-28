#include "private/muk.pch"
#include "QuickPlanningConfigurator.h"

#include "MukCommon/PlannerFactory.h"

namespace gris
{
namespace muk
{
  /**
  */
  QuickPlanningConfigurator QuickPlanningConfigurator::create(const std::string& plannerName)
  {
    QuickPlanningConfigurator ret;
    if (plannerName=="Spline-Based-RRT")
    {
      ret.mPlanner = plannerName;
      ret.mPruner  = "Yang-Spiral-Pruner";
      ret.mInterpolator = "Yang-Spiral-Interpolator";
      ret.mSphereResolution = 20;
      ret.mNewPaths = 20;
    }
    else if (plannerName=="Linear-Planner")
    {
      ret.mPlanner = plannerName;
      ret.mPruner  = "PrunerDummy";
      ret.mInterpolator = "Linear-Interpolator";
      ret.mSphereResolution = 80;
      ret.mNewPaths = 1;
    }
    else
    {
      ret.mPlanner = plannerName;
      ret.mPruner  = "PrunerDummy";
      ret.mInterpolator = "Linear-Interpolator";
      ret.mSphereResolution = 20;
      ret.mNewPaths = 20;
    }
    ret.mCalcTime = 0.25;
    ret.mStepSize = 2.0;
    return ret;
  }
}
}