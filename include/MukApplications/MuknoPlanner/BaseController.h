#pragma once

#include "qobject.h"

namespace gris
{
  namespace muk
  {
    struct AppControllers;
    struct AppModels;
    class MuknoPlannerMainWindow;

    /**
    */
    class BaseController : public QObject
    {
      Q_OBJECT

      public:
        virtual ~BaseController() {}

        void setControllers(AppControllers* pObj);
        void setModels(AppModels* pObj);
        void setMainWindow(MuknoPlannerMainWindow* pWindow) { mpMainWindow = pWindow; }
        virtual void initialize() {}

      protected:
        AppControllers* mpControls;
        AppModels* mpModels;
        MuknoPlannerMainWindow* mpMainWindow;
    };
  }
}