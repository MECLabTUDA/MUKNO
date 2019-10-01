#include "private/muk.pch"
#include "PluginManagerPP.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/OptimizerFactory.h"
#include "MukCommon/InterpolatorFactory.h"

namespace gris
{
  namespace muk
  {
    void PluginManagerPP::initialize()
    {
      GetPlannerFactory();
      GetInterpolatorFactory();
      GetOptimizerFactory();
    }
  }
}