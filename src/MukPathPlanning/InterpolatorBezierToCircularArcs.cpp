#include "private/muk.pch"
#include "InterpolatorBezierToCircularArcs.h"

#include "private/magic_numbers.h"
#include "BezierToArcTrajectory.h"
#include "BezierSpline.h"

#include "MukCommon/InterpolatorFactory.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukProblemDefinition.h"

namespace 
{
  using namespace gris;
  using namespace gris::muk;
  void makeStableNormal(const Vec3d& v, const Vec3d& w, Vec3d& normal);

  std::vector<MukState> toStates(const std::vector<Vec3d>& points)
  {
    std::vector<MukState> result;
    std::transform(points.begin(), points.end(), std::back_inserter(result), [&](const auto& p) { return MukState(p, Vec3d()); });
    if (points.size() > 1)
      result[0].tangent = (result[1].coords - result[0].coords).normalized();
    for (size_t i(1); i<result.size(); ++i)
      result[i].tangent = (result[i].coords - result[i-1].coords).normalized();
    return result;
  }

  
}

namespace gris
{
namespace muk
{
  REGISTER_INTERPOLATOR(InterpolatorBezierToCircularArcs);

  // ---------------------------------------------------------------
  /**
  */
  struct InterpolatorBezierToCircularArcs::Impl
  {
    std::unique_ptr<BezierToArcTrajectory> pTrajectory;
  };

  // ===============================================================

  /**
  */
  InterpolatorBezierToCircularArcs::InterpolatorBezierToCircularArcs()
    : mp(std::make_unique<Impl>())
  {
  }

  // ---------------------------------------------------------------
  /**
  */
  InterpolatorBezierToCircularArcs::~InterpolatorBezierToCircularArcs()
  {
  }

  // ---------------------------------------------------------------
  /** \brief Computes a circular arc representation, throws if it is in collision.
  */
  void InterpolatorBezierToCircularArcs::interpolate()
  {
    if (mInput.getStates().empty())
    {
      LOG_LINE << __FUNCTION__ << ": Unable to interpolate! The input MukPath has no states!";
      return;
    }
    mInterpolated = true;

    mp->pTrajectory = std::make_unique<BezierToArcTrajectory>();
    mp->pTrajectory->setKappa(mKappa);
    
    std::vector<Vec3d> points;
    const auto& states = mInput.getStates();
    std::transform(states.begin(), states.end(), std::back_inserter(points), [&](const auto& q) { return q.coords; });

    mp->pTrajectory->setWaypoints(points);
    mp->pTrajectory->setHints(mInput.getCircularTypes());
    mp->pTrajectory->compute();
  }

  /**
  */
  IInterpolator::result_type InterpolatorBezierToCircularArcs::getControlPoints() const
  {
    IInterpolator::result_type result;
    result.setRadius(mInput.getRadius());
    const auto& states = toStates(mp->pTrajectory->getWaypoints());
    result.setStates(states);
    return result;
  }

  /**
  */
  std::vector<bool> InterpolatorBezierToCircularArcs::validStates() const
  {
    return std::vector<bool>();
  }
  
  /**
  */
  IInterpolator::result_type InterpolatorBezierToCircularArcs::getInterpolation(EnInterpolationTypes type) const
  {
    IInterpolator::result_type result;
    result.setRadius(mInput.getRadius());
    std::vector<Vec3d> points;
    switch (type)
    {
      case controlPoints:
      {
        points = mp->pTrajectory->getWaypoints();
        break;
      }
      case samples:
      {
        points = mp->pTrajectory->getSamplesPoints();
        break;
      }
      case pointsPerSegment:
      {          
        points = mp->pTrajectory->getResolutionPoints(mResolution);
        break;
      }
      case resolution:
      {
        points = mp->pTrajectory->getResolutionPoints(mResolution);
        break;          
      }
    }
    // compute direction / tangent
    const auto& states = toStates(points);
    result.setStates(states);
    return result;
  }

  /**
  */
  IInterpolator::InterpolatedPoses InterpolatorBezierToCircularArcs::getInterpolatedPoses(EnInterpolationTypes type) const
  {
    InterpolatedPoses result;
    // for future refactoring
    //switch (type)
    //{
    //  /*case controlPoints:
    //  {
    //    result = mp->pTrajectory->getWaypoints();
    //    break;
    //  }*/
    //  case samples:
    //  {
    //    result = mp->pTrajectory->getSamplesPoints();
    //    break;
    //  }
    //  case pointsPerSegment:
    //  {          
    //    result = mp->pTrajectory->getResolutionPoints(mResolution);
    //    break;
    //  }
    //  case resolution:
    //  {
    //    result = mp->pTrajectory->getResolutionPoints(mResolution);
    //    break;          
    //  }
    //}
    // now use only control points
    result = mp->pTrajectory->getSamples();
    return result;
  }  
}
}

namespace
{
  /** \brief updates the normal if it was created from cotangent vectors
  */
  void makeStableNormal(const Vec3d& v, const Vec3d& w, Vec3d& normal)
  {
    while (normal.hasNaN())
    {
      const auto x = std::rand() % 10; // the range 0 to 9
      const auto y = std::rand() % 10; // the range 0 to 9
      const auto z = std::rand() % 10; // the range 0 to 9
      normal = Vec3d(x,y,z).normalized();
      normal = v.cross(normal);
    }
  }
}