#pragma once

#include "MukPathPlanning/BezierSpline.h"

#include <MukCommon/muk_common.h>

#include <Eigen/Dense>

namespace gris
{
  namespace muk
  {
    /** \brief Interpolator as proposed by Yang et al.

      see Yang et al, An analytical continuous-curvature path-smoothing algorithm, 2010, IEEE

      No analytical solution for computing gamma from kappa and minLength available
    */
    class BezierSpiralInterpolator
    {
      public:
        // determine the third parameter from the other two
        static double fromKappaAndGamma(double kappa, double gamma);
        static double fromKappaAndMinLength(double kappa, double minLength);
        static double fromMinLengthAndGamma(double minLength, double gamma);

      public:
        BezierSpiralInterpolator();
        ~BezierSpiralInterpolator() {}

      public:
        void    setKappa(double val)        { mKappa = val; }
        double  getKappa()            const { return mKappa; }
        void    setGamma(double val);
        double  getGamma()            const { return mGamma; }
        void    setMinLength(double val)    { mMinLength = val; }
        double  getMinLength()        const { return mMinLength; }

        bool interpolatable(const Vec3d& W1, const Vec3d& W2, const Vec3d W3) const;
                            
        bool interpolate   (const Eigen::Vector3d& W1, const Eigen::Vector3d& W2, const Eigen::Vector3d& W3, BezierSpline& spline) const;
        bool interpolate   (const Vec3d& W1, const Vec3d& W2, const Vec3d& W3, BezierSpline& spline) const;
        
      private:
        double mKappa;
        double mGamma;
        double mMinLength;
    };
  }
}
