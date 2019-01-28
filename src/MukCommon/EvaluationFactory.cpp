#include "private/muk.pch"
#include "EvaluationFactory.h"

#include <loki/Singleton.h>

namespace gris
{
namespace muk
{
  typedef Loki::SingletonHolder<EvaluationFactory, Loki::CreateUsingNew > TheEvaluationFactory;

  EvaluationFactory& GetEvaluationFactory()
  {
    return TheEvaluationFactory::Instance();
  }
}
}
