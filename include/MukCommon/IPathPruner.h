#pragma once

#include "muk_common_api.h"

#include "ICollisionDetector.h"
#include "MukPath.h"

#include "gstd/dynamicProperty.h"

#include <memory>

namespace gris
{
  namespace muk
  {
    class MUK_COMMON_API IPathPruner : public gstd::DynamicProperty
    {
      public:
        IPathPruner();
        virtual ~IPathPruner() {}

      public:
        virtual const char* name() const = 0;

      public:
        void setCollisionDetector(std::shared_ptr<ICollisionDetector> pObj) { mpCollision = pObj; }
        std::shared_ptr<ICollisionDetector> getCollisionDetector() const { return mpCollision; }

        void setKappa(double d)       { mKappa = d; }
        void setMaxDistance(double d) { mMaxDist = d; } /** \brief sets the maximum distance between two states */
        void setMaxStepSize(double d) { mMaxStepSize = d; }
        double getKappa()       const { return mKappa; }
        double getMaxDistance() const { return mMaxDist; }
        double getMaxStepSize() const { return mMaxStepSize; }
      
      public:
        virtual void clone(const IPathPruner* pOther) = 0;

      public:
        virtual MukPath calculate(const MukPath& input) = 0;

      protected:
        std::shared_ptr<ICollisionDetector> mpCollision;
        double mMaxDist;     // minimal distance to risc structures
        double mMaxStepSize; // maximal distance to the previous state
        double mKappa;       // maximum curvature constraint
    };
  }
}

