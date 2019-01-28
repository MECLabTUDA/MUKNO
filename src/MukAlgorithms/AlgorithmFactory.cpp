#include "private/muk.pch"
#include "AlgorithmFactory.h"

#include <loki/Singleton.h>

namespace gris
{
namespace muk
{
  typedef Loki::SingletonHolder<AlgorithmFactory, Loki::CreateUsingNew > TheAlgorithmFactory;

  AlgorithmFactory& GetAlgorithmFactory()
  {
    return TheAlgorithmFactory::Instance();
  }
}
}