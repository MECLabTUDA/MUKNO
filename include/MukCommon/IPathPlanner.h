#pragma once

#include "muk_common_api.h"

#include "IBounds.h"
#include "ICollisionDetector.h"
#include "MukPath.h"
#include "MukPathGraph.h"
#include "MukState.h"
#include "StateSamplerData.h"

#include "gstd/DynamicProperty.h"

#include <memory>

namespace gris
{
namespace muk
{
  class MukProblemDefinition;

  /** \brief Interface for the path-planning procedure
    
    Implemented classes should internally define and configure an ompl::base::Planner
    Interface development in progress, some things are not need / used (e.g. numPathsToFind)
  */
  class MUK_COMMON_API IPathPlanner : public gstd::DynamicProperty
  {
    public:
      IPathPlanner();
      virtual ~IPathPlanner() {}

      IPathPlanner(const IPathPlanner&)            = delete;
      IPathPlanner& operator=(const IPathPlanner&) = delete;
      IPathPlanner(IPathPlanner&&)                 = default;
      IPathPlanner& operator=(IPathPlanner&&)      = default;

    public:
      virtual const char* name() const = 0;
      
    public:
      virtual void            setStepSize(double val)                   { mStepSize = val; }
      virtual double          getStepSize()                     const   { return mStepSize; }
      void                    setCalculationTime(double val)            { mCalculationTime = val;  }
      double                  getCalculationTime()              const   { return mCalculationTime; }
      
      virtual void setCollisionDetector(std::shared_ptr<ICollisionDetector> pObj) { mpCollisionDetector = pObj; }
      std::shared_ptr<ICollisionDetector> getCollisionDetector()  const   { return mpCollisionDetector; }

    public:
      void    clone(const IPathPlanner* pOther);

    public:
      virtual void             initialize()  = 0;  // create basic data (e.g. ompl data strcutures)
      virtual bool             calculate()   = 0;  // continue solving
      virtual void             update()      = 0;  // reset parameters    
      virtual void             clearSearch() = 0;  // clear previously done search
      virtual size_t           availablePaths() const = 0;

      void                                  setProblemDefinition(std::weak_ptr<MukProblemDefinition> pObj);
      std::shared_ptr<MukProblemDefinition> getProblemDefinition();
      
    public:
      virtual MukPath        extractPath(size_t idx=0) const = 0;
      virtual MukPathGraph   extractRoadmap()       const = 0;

    protected:
      double    mCalculationTime; // seconds
      double    mStepSize;

      std::shared_ptr<ICollisionDetector> mpCollisionDetector;
      std::weak_ptr<MukProblemDefinition> mpProbDef;
  };

}
}

