#pragma once
#include "muk_evaluation_api.h"

#include <string>
#include <vector>

namespace gris
{
  namespace muk
  {
    class MukPath;
    class ICollisionDetector;

    /**
    */
    class MUK_EVAL_API CollisionDetectionHandler
    {
      public:
        CollisionDetectionHandler();
        ~CollisionDetectionHandler();

      public:
        void initialize(ICollisionDetector* pObj);
        void setupForObstacle(const std::string& key);
        std::pair<size_t, double> distance(const MukPath& path, double pathRadius);
        void finish();

      private:
        std::vector<std::pair<std::string, bool>> mPreSetup;
        ICollisionDetector* pDetector;
    };
  }
}
