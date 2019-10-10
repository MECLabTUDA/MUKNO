#include "private/muk.pch"
#include "BaseController.h"

#include "AppControllers.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    void BaseController::setControllers(AppControllers* pObj)
    {
      mpControls = pObj;
    }

    /**
    */
    void BaseController::setModels(AppModels* pObj)
    {
      mpModels = pObj;
    }
  }
}