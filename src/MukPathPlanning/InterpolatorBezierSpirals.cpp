#include "private/muk.pch"
#include "InterpolatorBezierSpirals.h"

#include "private/BezierSpiral3DInterpolator.h"

#include "MukCommon/MukException.h"
#include "MukCommon/InterpolatorFactory.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace
{
}

namespace gris
{
namespace muk
{  
  REGISTER_INTERPOLATOR(InterpolatorBezierSpirals);

  /**
  */
  struct InterpolatorBezierSpirals::Impl
  {
    Impl();
    ~Impl() {}

    void getSamples         (std::vector<Vec3d>& v);
    void getDiscretization  (std::vector<Vec3d>& v, size_t mSamples);
    void getResolution      (std::vector<Vec3d>& v, double resolution);    

    BezierSpiral3DInterpolator interpolator;
    std::vector<BezierSpline>  splines;
    std::vector<bool>          splinesFlagsValid;
  };

  InterpolatorBezierSpirals::Impl::Impl()    
  {
  }

  // ----------------------------------------------------------------------------------------
  
  /**
  */
  InterpolatorBezierSpirals::InterpolatorBezierSpirals()
    : IInterpolator()
    , mp(std::make_unique<Impl>())
  {
  }

  /**
  */
  InterpolatorBezierSpirals::~InterpolatorBezierSpirals()
  {
  }

  /**
  */
  bool InterpolatorBezierSpirals::isValid() const
  {
    return true;
  }

  /**
  */
  void InterpolatorBezierSpirals::setKappa(double kappa)
  {
    mKappa = kappa;
  }
  
  /**
  */
  double InterpolatorBezierSpirals::getKappa() const
  {
    return mKappa;
  }

  /**
  */
  void InterpolatorBezierSpirals::interpolate()
  {
    mInterpolated = true;
    // for each two states: compute intersection of tangents, then create the splines if possible
    const auto& states = mInput.getPath();
    const size_t N = states.size()-1;
    mp->splines.resize(N);
    mp->splinesFlagsValid.resize(N);

    for (size_t i(1); i<=N; ++i)
    {      
      bool valid = mp->interpolator.interpolate(states[i-1], states[i], mp->splines[i-1]);
      mp->splinesFlagsValid[i-1] = valid;
    }        
  }

  /**
  */
  IInterpolator::result_type InterpolatorBezierSpirals::getControlPoints() const
  {
    IInterpolator::result_type result;
    result.setRadius(mInput.getRadius());
    result.getPath().reserve( mInput.getPath().size() );
    std::copy(mInput.getPath().begin(), mInput.getPath().end(), back_inserter(result.getPath()));
    return result;
  }

  /**
  */
  IInterpolator::result_type InterpolatorBezierSpirals::getInterpolation(EnInterpolationTypes type) const
  {    
    std::vector<Vec3d> points;
    switch (type)
    {
      case controlPoints:
      {
        std::transform(mInput.getPath().begin(), mInput.getPath().end(), back_inserter(points), [&] (const MukState& state) { return state.coords; });
        break;
      }
      case samples:
      {
        mp->getSamples(points);
        break;
      }
      case pointsPerSegment:
      {
        mp->getDiscretization(points, mPointsPerSegment);
        break;
      }
      case resolution:
      {
        mp->getResolution(points, mResolution);
        break;
      }
      default:
      {
        LOG_LINE << __FUNCTION__ << " Interpolation type not implemented!";
      }
    }

    IInterpolator::result_type result;
    result.setRadius(mInput.getRadius());
    const size_t N = points.size();    
    auto& path     = result.getPath();
    path.reserve(N);
    if (N >= 2)
    {  
      Vec3d tangent = path[1].coords-path[0].coords;
      tangent.normalize();
      path.push_back(MukState(points[0], tangent));
      for (size_t i(1); i<N; ++i)
      {
        tangent = points[i] - points[i-1];
        tangent.normalize();
        path.push_back(MukState(points[i], tangent));
      }
    }
    return result;
  }

  /**
    needed to color the line according to valid/invalid interpolation.
    needs to have the same size as result of call to getInterpolation()
    -> add an additional point
  */
  std::vector<bool> InterpolatorBezierSpirals::validStates() const
  {
    std::vector<bool> vecValid;
    const size_t N = mp->splines.size();

    switch (mInterpolationType)
    {
      case controlPoints:
      case pointsPerSegment:
      case resolution:
      {
        vecValid.resize(N+1, true);
        for (size_t i(0); i<N; ++i)
        {
          vecValid[i] = false;
        }
          vecValid.back() = false;
        break;
      }
      case samples:
      {
        vecValid.reserve(N*7+1);
        for (size_t i(0); i<N; ++i)
        {
          for(int i(0); i<6;++i)
            vecValid.push_back(true);
        }
        break;
      }
      default:
      {
        LOG_LINE << __FUNCTION__ << " Interpolation type not implemented!";
      }
    }        
    return vecValid;
  }

  // ------------------------------------------------------------------
  
  /**
  */
  void InterpolatorBezierSpirals::Impl::getSamples(std::vector<Vec3d>& v)
  {
    v.reserve(splines.size() * 6 + 1);
    for (size_t i(0); i<splines.size(); ++i)
    {
      std::copy(splines[i].samples.begin(), splines[i].samples.end()-1, std::back_inserter(v));
    }
    if ( ! v.empty())
      v.push_back(splines.back().sample(7));
  }

  /**
  */
  void InterpolatorBezierSpirals::Impl::getDiscretization(std::vector<Vec3d>& v, size_t mSamples)
  {
    v.reserve(splines.size() * 6 + 1);
    for (size_t i(0); i<splines.size(); ++i)
    {
      std::copy(splines[i].samples.begin(), splines[i].samples.end()-1, std::back_inserter(v));
    }
  }

  /**
  */
  void InterpolatorBezierSpirals::Impl::getResolution(std::vector<Vec3d>& v, double resolution)
  {
    for (size_t i(0); i<splines.size(); ++i)
    {
      std::vector<Vec3d> tmp;
      splines[i].uniformSamples(resolution, tmp);
      std::copy(tmp.begin(), tmp.end(), std::back_inserter(v));
    }
  }

}
}

namespace
{
}