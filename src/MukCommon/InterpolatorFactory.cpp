#include "private/muk.pch"
#include "InterpolatorFactory.h"

#include <loki/Singleton.h>

namespace gris
{
  namespace muk
  {
    typedef Loki::SingletonHolder<InterpolatorFactory, Loki::CreateUsingNew > TheInterpolatorFactory;

    InterpolatorFactory& GetInterpolatorFactory()
    {
      return TheInterpolatorFactory::Instance();
    }
  }
}
