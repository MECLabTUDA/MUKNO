#include "private/muk.pch"
#include "OptimizerFactory.h"

#include <loki/Singleton.h>

namespace gris
{
  namespace muk
  {
    typedef Loki::SingletonHolder<OptimizerFactory, Loki::CreateUsingNew > TheOptimizerFactory;

    OptimizerFactory& GetOptimizerFactory()
    {
      return TheOptimizerFactory::Instance();
    }
  }
}
