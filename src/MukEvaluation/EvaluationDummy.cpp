#include "private/muk.pch"
#include "EvaluationDummy.h"

#include "MukCommon/EvaluationFactory.h"

namespace gris
{
  namespace muk
  {
    REGISTER_EVALUATION(EvaluationDummy);
    /**
    */
    void EvaluationDummy::evaluate()
    {
      LOG_LINE << __FUNCTION__ << " evaluated:\n"
        << "   " << mQuery;
    }
  }
}