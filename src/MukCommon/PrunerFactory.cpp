#include "private/muk.pch"
#include "PrunerFactory.h"

#include <loki/Singleton.h>

namespace gris
{
  namespace muk
  {
    typedef Loki::SingletonHolder<PrunerFactory, Loki::CreateUsingNew > ThePrunerFactory;

    PrunerFactory& GetPrunerFactory()
    {
      return ThePrunerFactory::Instance();
    }
  }
}
