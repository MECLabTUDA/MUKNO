#include "private/muk.pch"
#include "PlannerFactory.h"

#include <loki/Singleton.h>

namespace gris
{
  namespace muk
  {
    typedef Loki::SingletonHolder<PlannerFactory, Loki::CreateUsingNew > ThePlannerFactory;

    PlannerFactory& GetPlannerFactory()
    {
      return ThePlannerFactory::Instance();
    }
  }
}
