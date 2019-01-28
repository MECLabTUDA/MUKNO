#include "private/muk.pch"
#include "InterpolatorLinear.h"

#include "MukCommon/InterpolatorFactory.h"

#include "MukCommon/MukException.h"

namespace gris
{
namespace muk
{
  REGISTER_INTERPOLATOR(InterpolatorLinear);
  
  /**
  */
  void InterpolatorLinear::interpolate()
  { 
    if (mInput.getPath().empty())
    {
      LOG_LINE << __FUNCTION__ << ": Unable to interpolate! The input MukPath has no states!";
      return;
    }     
    mInterpolated = true;
  }
    
  /**
  */
  bool InterpolatorLinear::isValid() const
  {
    return true;
  }

  /**
  */
  IInterpolator::result_type InterpolatorLinear::getControlPoints() const
  {
    return mInput;
  }

  /**
  */
  IInterpolator::result_type InterpolatorLinear::getInterpolation(EnInterpolationTypes type) const
  {
    if ( ! mInterpolated )
      throw MUK_EXCEPTION_SIMPLE("The Interpolator needs te be updated first!");    

    IInterpolator::result_type result;
    
    const auto&  input   = mInput.getPath();
    const size_t N       = input.size();
    auto&        path    = result.getPath();
    mNumelResult = 0;
    switch (type)
    {
      case controlPoints:
      case samples:
      {
        result = getControlPoints();
        mNumelResult = result.getPath().size();
        break;
      }
      case pointsPerSegment:
      {        
        result.setRadius(mInput.getRadius());
        path.reserve( (N-1)* mPointsPerSegment + 1);
        const double stepSize = 1.0 / mPointsPerSegment;
        for (size_t i(1); i<N; ++i)
        {
          Vec3d t1 = input[i].coords - input[i-1].coords;          
          for (size_t j(0); j <mPointsPerSegment; ++j)
          {
            MukState next;
            next.coords  = input[i-1].coords  + j*stepSize*t1;
            next.tangent = input[i-1].tangent;
            path.push_back(next);
          }
        }
        path.push_back(input.back());
        mNumelResult = path.size();
        break;
      }
      case resolution:
      {
        result.setRadius(mInput.getRadius());        
        for (size_t i(1); i<N; ++i)
        {
          Vec3d t1 = input[i].coords - input[i-1].coords;         
          const size_t N_Steps = static_cast<size_t>( t1.norm() / mResolution + 0.5 );
          t1.normalize();
          for (size_t j(0); j < N_Steps; ++j)
          {
            MukState next;
            next.coords  = input[i-1].coords + j*mResolution*t1;
            path.push_back(next);
          }
        }        
        path.push_back(input.back());
        mNumelResult = path.size();
        break;
      }
      default:
      {
        LOG_LINE << __FUNCTION__ << " Interpolation type not implemented!";
      }
    }
    
    // set tangents / directions
    path[0].tangent = path[1].coords - path[0].coords;
    path[0].tangent.normalize();
    for (size_t i(1); i<path.size(); ++i)
    {
      path[i].tangent = path[i].coords - path[i-1].coords;
      path[i].tangent.normalize();
    }

    return result;
  }

  /**
  */
  std::vector<bool> InterpolatorLinear::validStates() const
  {
    std::vector<bool> result(mNumelResult, true);
    return result;
  }
}
}