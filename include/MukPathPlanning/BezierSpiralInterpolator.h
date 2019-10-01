#pragma once
#include "BezierSpline.h"
#include "muk_pathplanning_api.h"

namespace gris
{
  namespace muk
  {
    /** \brief Interpolator as proposed by Yang et al.

      see Yang et al, An analytical continuous-curvature path-smoothing algorithm, 2010, IEEE

      No analytical solution for computing gamma from kappa and minLength available
    */
    class MUK_PP_API BezierSpiralInterpolator
    {
      public:
        // determine the third parameter from the other two
        static double fromKappaAndGamma(double kappa, double gamma);
        static double fromKappaAndMinLength(double kappa, double minLength);
        static double fromMinLengthAndGamma(double minLength, double gamma);
        static void   approximateStepSize(double eps, double minGamma, double maxGamma, double stepSize, gris::muk::BezierSpiralInterpolator& spiralBuilder);

      public:
        static BezierSpline createFromWaypoints(const Vec3d& W1, const Vec3d& W2, const Vec3d W3);
        static BezierSpline createFromEndpoints(const Vec3d& B0, const Vec3d& W2, const Vec3d E0);

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
        bool interpolate   (const Vec3d& W1, const Vec3d& W2, const Vec3d& W3, BezierSpline& spline) const;
        
        double computeKappaMax(const Vec3d& W1, const Vec3d& W2, const Vec3d& W3) const;
        
      private:
        double mKappa;
        double mGamma;
        double mMinLength;
    };
  }
}
