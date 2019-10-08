#include "private/muk.pch"
#include "CalibratorDummy.h"

#include "MukCommon/CalibrationFactory.h"
#include "MukCommon/MukException.h"

namespace gris
{
  namespace muk
  {
    REGISTER_CALIBRATION_STATEMACHINE(CalibratorDummy);
    
    CalibratorDummy::CalibratorDummy()
      : CalibrationStateMachine()
    {
      registerCalibrationStep(std::string("The Dummy really does not do anything."));
      setCountDownTicks(0);
    }

  }
}