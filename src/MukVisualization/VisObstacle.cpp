#include "private/muk.pch"
#include "VisObstacle.h"

namespace gris
{
namespace muk
{
  /**
  */
  VisObstacle::VisObstacle(const MukObstacle& obs)
    : VisAbstractObject(obs.getName())
  {
  }

  /**
  */
  VisObstacle::~VisObstacle()
  {
  }
}
}