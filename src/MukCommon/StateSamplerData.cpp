#include "private/muk.pch"
#include "StateSamplerData.h"

namespace gris
{
namespace muk
{
  /**
  */
  void StateSamplerData::setData(StateSamplerType type, const Bounds& bounds, const std::vector<Vec3d>& list)
  {
    mType = type;
    mBounds = bounds;
    mPointList = list;
  }

  /**
  */
  void StateSamplerData::setData(StateSamplerType type, const Bounds& bounds)
  {
    mType = type;
    mBounds = bounds;
  }

  /**
  */
  std::string StateSamplerData::toString(StateSamplerType type)
  {
    std::string result;
    switch(type)
    {
      case enBBox:
        result = "BoundingBox";
        break;
      case enPointList: 
        result = "PointList";
        break;
    };
    return result;
  }
}
}
