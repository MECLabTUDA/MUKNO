#pragma once

#include "Bounds.h"
#include "muk_common_api.h"

#include <vector>
#include <string>

namespace gris
{
  namespace muk
  {
    /** \brief contains all data to create a certain kind of ompl state sampler

      the current type's necessary data has to be set
    */
    class MUK_COMMON_API StateSamplerData
    {
      public:
        enum StateSamplerType
        {
          enBBox,
          enPointList
        };

        static std::string toString(StateSamplerType);
        
      public:
        void setData(StateSamplerType type, const Bounds& bounds, const std::vector<Vec3d>& list);
        void setData(StateSamplerType type, const Bounds& bounds);

        StateSamplerType getType() const { return mType; }

        const Bounds&              getBounds()    const { return mBounds; };
        const std::vector<Vec3d>&  getPointList() const { return mPointList; };
        
      private:
        std::vector<Vec3d> mPointList;
        Bounds             mBounds;
        StateSamplerType   mType = enBBox;
    };
  }
}
