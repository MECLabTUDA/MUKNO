#pragma once

#include "BaseController.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class ToolBarController : public BaseController
    {
      Q_OBJECT

      signals:
        void activePathIndexChanged();

        void pathAdded            (const std::string& key, size_t idx);
        void pathChanged          (const std::string& key, size_t idx);

        void interpolated(const std::string& key);
        void extractRoadMap();

        void ctFileLoaded();
        void sceneCleared();

      public:
        void setupConnnections();

        void focusOnDefault();
        void focusOnWaypoint();
        void clearScene();
        void loadScene(const std::string& filename);
        void loadCTFile(const std::string& filename);
        void loadSegmentation(const std::string& filename);
        void loadObstacleFromFile(const std::string& filename);
        void saveScene(const std::string& filename);

      private:
    };
  }
}
