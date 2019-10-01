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
    class MukProblemDefinition;

    /**
    */
    class MUK_COMMON_API IPathOptimizer : public gstd::DynamicProperty
    {
      public:
        IPathOptimizer();
        virtual ~IPathOptimizer() {}

      public:
        virtual const char* name() const = 0;

      public:
        void                                  setCollisionDetector(std::shared_ptr<ICollisionDetector> pObj) { mpCollision = pObj; }
        std::shared_ptr<ICollisionDetector>   getCollisionDetector() const { return mpCollision; }
        void                                  setProblemDefinition(std::weak_ptr<MukProblemDefinition> pObj);
        std::shared_ptr<MukProblemDefinition> getProblemDefinition();

        void setKappa(double d)       { mKappa = d; }
        void setMaxDistance(double d) { mMaxDist = d; } /** \brief sets the maximum distance between two states */
        void setMaxStepSize(double d) { mMaxStepSize = d; }
        double getKappa()       const { return mKappa; }
        double getMaxDistance() const { return mMaxDist; }
        double getMaxStepSize() const { return mMaxStepSize; }
        
        bool    valid() const { return mValid; }
      
      public:
        virtual void clone(const IPathOptimizer* pOther);

      public:
        virtual MukPath calculate(const MukPath& input) const = 0;

      protected:
        std::weak_ptr<MukProblemDefinition> mpProbDef;
        std::shared_ptr<ICollisionDetector> mpCollision;
        double mMaxDist;     // minimal distance to risc structures
        double mMaxStepSize; // maximal distance to the previous state
        double mKappa;       // maximum curvature constraint
        mutable bool mValid;
    };
  }
}

