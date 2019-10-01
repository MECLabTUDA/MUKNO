#include "private/muk.pch"
#include "IPathOptimizer.h"
#include "MukProblemDefinition.h"
#include "MukException.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    IPathOptimizer::IPathOptimizer()
      : mMaxDist(0)
      , mMaxStepSize(0)
      , mKappa(0)
      , mValid(false)
    {
      declareProperty<double>("MaximalDistance", MUK_SET(double, setMaxDistance), MUK_GET(getMaxDistance));
      declareProperty<double>("MaxStepSize",     MUK_SET(double, setMaxStepSize), MUK_GET(getMaxStepSize));
      declareProperty<double>("Curvature",       MUK_SET(double, setKappa),       MUK_GET(getKappa));
    }

    /**
    */
    void IPathOptimizer::clone(const IPathOptimizer* pOther)
    {
      setKappa(pOther->getKappa());
      setMaxDistance(pOther->getMaxDistance());
      setMaxStepSize(pOther->getMaxStepSize());
      mpCollision = pOther->mpCollision;
    }

    /**
    */
    void IPathOptimizer::setProblemDefinition(std::weak_ptr<MukProblemDefinition> pObj)
    {
      mpProbDef = pObj;
    }

    /**
    */
    std::shared_ptr<MukProblemDefinition> IPathOptimizer::getProblemDefinition()
    {
      auto res = mpProbDef.lock();
      if (!res)
      {
        throw MUK_EXCEPTION_SIMPLE("ProblemDefinition no longer valid");
      }
      return res;
    }
  }
}