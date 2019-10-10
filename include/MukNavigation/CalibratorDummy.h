#pragma once

#include "muk_navigation_api.h"
#include "MukCommon/CalibrationStateMachine.h"
#include "MukCommon/INavigationSupervisor.h"

#include <chrono>

namespace gris
{
  namespace muk
  {
    class CalibratorDummy;

/*    class CalibratorDummyProperty : public NavigatorProperty
    {
    public:
      NavigatorDummyProperty(NavigatorDummy* nav);
    };*/

    /**
    */
    class MUK_NAVI_API CalibratorDummy : public CalibrationStateMachine
    {
    public:
      CalibratorDummy();

    public:
      static  const char* s_name() { return "CalibratorDummy"; }
      virtual const char* name()   const { return s_name(); }

      static  const char* s_printableName() { return "Dummy Calibration Algorithm"; }
      virtual const char* printableName()    const { return s_printableName(); }
    };

  }
}