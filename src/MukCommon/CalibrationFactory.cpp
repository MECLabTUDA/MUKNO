#include "private/muk.pch"
#include "CalibrationFactory.h"

#include <loki/Singleton.h>

namespace gris
{
  namespace muk
  {
    typedef Loki::SingletonHolder<CalibrationFactory, Loki::CreateUsingNew > TheCalibrationFactory;

    CalibrationFactory& GetCalibrationFactory()
    {
      return TheCalibrationFactory::Instance();
    }
  }
}
