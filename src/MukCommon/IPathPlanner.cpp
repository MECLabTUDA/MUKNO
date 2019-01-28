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
    : mCalculationTime(0.25), mStepSize(2.0)
  {
    declareProperty<double>("Calc-Time",
      std::bind(&IPathPlanner::setCalculationTime, this, std::placeholders::_1),
      std::bind(&IPathPlanner::getCalculationTime, this));
    declareProperty<double>("Step-Size",
      std::bind(&IPathPlanner::setStepSize, this, std::placeholders::_1),
      std::bind(&IPathPlanner::getStepSize, this));
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