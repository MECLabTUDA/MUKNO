#pragma once

#include "BaseController.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MenuBarController : public BaseController
    {
      Q_OBJECT 

      public:
        void setupConnnections();
        virtual void initialize();

      signals:
        void obstacleAdded  (const std::string& name);
        void obstacleDeleted(const std::string& name);   
        void propertyChanged(const char* key);

      public:
        void loadScene(const std::string& filename);
        void setDefaultFocus();
        void savePath(const std::string& filename);
        void loadPath(const std::string& filename);

        void evaluateQuickPlanningConfiguration();

      private:
    };
  }
}
