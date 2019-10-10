#pragma once

#include "BaseController.h"

#include <qthread.h>

#include <mutex>
#include <memory>

namespace gris
{
  namespace muk
  {
    class ThreadedController : public QThread
    {
      Q_OBJECT

      public:
        explicit ThreadedController(QObject* parent = Q_NULLPTR)
          : QThread(parent)
        {
        }
        virtual ~ThreadedController() {}

        void setControllers(AppControllers* pObj)             { mpControls = pObj; }
        virtual void setModels(AppModels* pObj)               { mpModels = pObj; }
        void setMainWindow(MuknoPlannerMainWindow* pWindow)   { mpMainWindow = pWindow; }
        virtual void initialize() {}

      protected:
        AppControllers* mpControls;
        AppModels* mpModels;
        MuknoPlannerMainWindow* mpMainWindow;
    };
  }
}
