#include "private/muk.pch"
#include "PluginManagerPP.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/PrunerFactory.h"
#include "MukCommon/InterpolatorFactory.h"

namespace gris
{
  namespace muk
  {
    void PluginManagerPP::initialize()
    {
      GetPlannerFactory();
      GetInterpolatorFactory();
      GetPrunerFactory();
    }
  }
}