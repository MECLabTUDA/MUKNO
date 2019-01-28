#include "private/muk.pch"
#include "NavigatorFactory.h"

#include <loki/Singleton.h>

namespace gris
{
  namespace muk
  {
    typedef Loki::SingletonHolder<NavigatorFactory, Loki::CreateUsingNew > TheNavigatorFactory;

    NavigatorFactory& GetNavigatorFactory()
    {
      return TheNavigatorFactory::Instance();
    }
  }
}
