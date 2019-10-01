#pragma once
#include "muk_convex_optimization_api.h"

#include "MukCommon/IPathOptimizer.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_CONV_OPT_API ConvexBezierSplineOptimizer : public IPathOptimizer
    {
      public:
        ConvexBezierSplineOptimizer();
        virtual ~ConvexBezierSplineOptimizer() {}

      public:
        static  const char* s_name()      { return "ConvexBezierSplineOptimizer"; }
        virtual const char* name() const  { return s_name();  }

      public:
        virtual MukPath calculate(const MukPath& input) const;

      public:
        double getMaxDistancePenalty()            const { return mMaxDistancePenalty; };
        void   setMaxDistancePenalty(double val)        { mMaxDistancePenalty = val; };

      private:
        double  mInitialTrustRegionSize;
        double  mMaxTrustRegionSize;
        double  mMinTrustRegionSize;

        double  mMaxDistancePenalty;

        bool    mUseDistanceConstraint;
        bool    mUseDistanceCost;
        bool    mUseCurvatureConstraint;
        bool    mUseCurvatureCost;
        bool    mUseLengthCost;
        bool    mVerbose;

        double mLengthWeight;
        double mObstacleWeight;
        double mCurvatureWeight;
    };
  }
}
