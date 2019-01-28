#pragma once

#include "MukPathPlanning/BezierSpline.h"

#include <MukCommon/muk_common.h>
#include <MukCommon/geometry.h>

#include <Eigen/Dense>

namespace gris
{
  namespace muk
  {
    class MukState;

    /** \brief Interpolator that connects to States in SE3.
        
    */
    class BezierSpiral3DInterpolator
    {
      public:
        // determine the third parameter from the other two
        static double fromKappaAndGamma(double kappa, double gamma);
        static double fromKappaAndMinLength(double kappa, double minLength);
        static double fromMinLengthAndGamma(double minLength, double gamma);

      public:
        BezierSpiral3DInterpolator();
        ~BezierSpiral3DInterpolator() {}

        public:
        void    setKappa(double val)        { mKappa = val; }
        double  getKappa()            const { return mKappa; }
        void    setGamma(double val);
        double  getGamma()            const { return mGamma; }
        void    setMinLength(double val)    { mMinLength = val; }
        double  getMinLength()        const { return mMinLength; }

        bool interpolatable(const MukState& from, const MukState& to) const;

        bool interpolate(const MukState& from, const MukState& to, BezierSpline& output);

      private:
        double mKappa;
        double mGamma;
        double mMinLength;
    };
  }
}
