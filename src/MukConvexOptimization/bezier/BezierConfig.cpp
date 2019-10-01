#include "bezier/BezierConfig.h"

#include "MukCommon/Quaternion.h"
#include "MukCommon/gris_math.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gris
{
  namespace bezier
  {
    /**
    */
    BezierConfig::BezierConfig(double kappa, double minLength)
    {
      mBuilder.setKappa(kappa);
      /*double minGamma = 0;
      double maxGamma = gris::M_Pi;
      double eps      = 0.01;*/
      //mBuilder.setGamma(maxGamma);
      const double gamma = muk::BezierSpiralInterpolator::fromKappaAndMinLength(kappa, minLength);
      mBuilder.setGamma(gamma);
      mBuilder.setMinLength(minLength);
      //muk::BezierSpiralInterpolator::approximateStepSize(eps, minGamma, maxGamma, minLength, mBuilder);
    }
    
    /**
    */
    void BezierConfig::setSpline(const Vec3d& p1, const Vec3d& p2,const Vec3d& p3)
    {
      setStartPoint(p1);
      setMiddlePoint(p2);
      setEndPoint(p3);
      computeSpline();
    }

    /**
    */
    void BezierConfig::computeSpline()
    { 
      mBuilder.interpolate(mStartPoint, mMiddlePoint, mEndPoint, mSpline); 
    }
  }
}