#include "private/muk.pch"
#include "IPathPlanner.h"

#include "Bounds.h"
#include "MukException.h"
#include "MukProblemDefinition.h"


namespace gris
{
namespace muk
{
  /**
  */
  IPathPlanner::IPathPlanner()
    : mCalculationTime(0.25)
    , mStepSize(2.0)
    , mVerbose(false)
  {
    declareProperty<double>("Calc-Time", MUK_SET(double, setCalculationTime), MUK_GET(getCalculationTime));
    declareProperty<double>("Step-Size", MUK_SET(double, setStepSize), MUK_GET(getStepSize));
    declareProperty<bool>  ("Verbose",   MUK_SET(bool,   setVerbose), MUK_GET(getVerbose));
  }

  /**
  */
  void IPathPlanner::setProblemDefinition(std::weak_ptr<MukProblemDefinition> pObj)
  {
    mpProbDef = pObj;
  }

  /**
  */
  std::shared_ptr<MukProblemDefinition> IPathPlanner::getProblemDefinition()
  {
    auto res = mpProbDef.lock();
    if (!res)
    {
      throw MUK_EXCEPTION_SIMPLE("ProblemDefinition no longer valid");
    }
    return res;
  }

  /**
  */
  void IPathPlanner::clone(const IPathPlanner* pOther)
  {
    if (nullptr != pOther)
    {
      this->setCollisionDetector(pOther->getCollisionDetector());
      this->setProblemDefinition(pOther->mpProbDef);
      this->setStepSize(pOther->getStepSize());
      this->setCalculationTime(pOther->getCalculationTime());
    }
  }
}
}