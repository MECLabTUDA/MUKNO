#include "private/muk.pch"
#include "IPathPruner.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    IPathPruner::IPathPruner()
      : mMaxDist(0)
      , mMaxStepSize(0)
      , mKappa(0)
    {
      declareProperty<double>("MaximalDistance",
        std::bind(&IPathPruner::setMaxDistance, this, std::placeholders::_1), 
        std::bind(&IPathPruner::getMaxDistance, this));
      declareProperty<double>("MaxStepSize",
        std::bind(&IPathPruner::setMaxStepSize, this, std::placeholders::_1), 
        std::bind(&IPathPruner::getMaxStepSize, this));
      declareProperty<double>("Curvature",
        std::bind(&IPathPruner::setKappa, this, std::placeholders::_1), 
        std::bind(&IPathPruner::getKappa, this));
    }
  }
}