#include "private/muk.pch"
#include "StateRegionFactory.h"

#include <loki/Singleton.h>

namespace gris
{
  namespace muk
  {
    typedef Loki::SingletonHolder<StateRegionFactory, Loki::CreateUsingNew > TheStateRegionFactory;

    StateRegionFactory& GetStateRegionFactory()
    {
      return TheStateRegionFactory::Instance();
    }
  }
}
