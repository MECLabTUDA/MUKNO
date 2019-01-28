#include "private/muk.pch"
#include "CollisionDetectionHandler.h"
#include "statistics.h"

#include "MukCommon/ICollisionDetector.h"
#include "MukCommon/MukPath.h"

namespace gris
{
namespace muk
{
  /**
  */
  CollisionDetectionHandler::CollisionDetectionHandler()
    : pDetector(nullptr)
  {
  }

  /**
  */
  CollisionDetectionHandler::~CollisionDetectionHandler()
  {
    finish();
  }

  /**
  */
  void CollisionDetectionHandler::initialize(ICollisionDetector* pObj)
  {
    pDetector = pObj;
    auto keys = pDetector->getKeys();
    for (const auto& key : keys)
    {
      mPreSetup.push_back(std::make_pair(key, pDetector->isActive(key)));
    }
  }

  /**
  */
  void CollisionDetectionHandler::setupForObstacle(const std::string& key)
  {
    std::for_each(mPreSetup.begin(), mPreSetup.end(), [&] (const auto& pair)
    {
      if (key == pair.first)
        pDetector->setActive(pair.first, true);
      else
        pDetector->setActive(pair.first, false);
    });
    pDetector->rebuild();
  }

  /**
  */
  std::pair<size_t, double> CollisionDetectionHandler::distance(const MukPath& path, double pathRadius)
  { 
    auto ret = std::make_pair(std::numeric_limits<size_t>::quiet_NaN(), std::numeric_limits<double>::infinity());
    if ( ! path.getPath().empty())
    {
      gris::Vec3d nn;
      auto dists = computeDistances(*pDetector, path.getPath(), pathRadius);
      auto iter = std::min_element(dists.begin(), dists.end());    
      ret.first = std::distance(dists.begin(), iter);
      ret.second = *iter;
    }
    return ret;
  }

  /**
  */
  void CollisionDetectionHandler::finish()
  {
    std::for_each(mPreSetup.begin(), mPreSetup.end(), [&] (const auto& pair)
    {
      pDetector->setActive(pair.first, pair.second);
    });
    pDetector->rebuild();
  }
}
}