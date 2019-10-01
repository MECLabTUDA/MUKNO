#pragma once
#include "bezier/bezier_types.h"

#include "MukPathPlanning/BezierSpline.h"
#include "MukPathPlanning/BezierSpiralInterpolator.h"

#include <memory>

namespace gris
{
  namespace bezier
  {
    /** \brief Just copies the translation from the solver, but transforms the orientation part
    */
    class BezierConfig
    {
      public:
        BezierConfig(double kappa, double minLength);

      public:
        void setStartVariable (const PointFromVariables& p) { mStartVariable = p; }
        void setMiddleVariable(const PointFromVariables& p) { mMiddleVariable = p; }
        void setEndvariable   (const PointFromVariables& p) { mGoalVariable = p; }
        void setStartPoint (const Vec3d& p) { mStartPoint = p; }
        void setMiddlePoint(const Vec3d& p) { mMiddlePoint = p; }
        void setEndPoint   (const Vec3d& p) { mEndPoint = p; }
        void setSpline(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3);
        void computeSpline();

        const Vec3d& getMiddlePoint() const { return mMiddlePoint; }
        const Vec3d& getEndPoint() const { return mEndPoint; }

        const muk::BezierSpline& getSpline() const { return mSpline; }
        double getGamma()               const { return mBuilder.getGamma(); }
        const muk::BezierSpiralInterpolator getBuilder()               const { return mBuilder; }

      private:
        Vec3d mStartPoint;
        Vec3d mMiddlePoint;
        Vec3d mEndPoint;
        PointFromVariables mStartVariable;
        PointFromVariables mMiddleVariable;
        PointFromVariables mGoalVariable;
        muk::BezierSpline mSpline;
        muk::BezierSpiralInterpolator mBuilder;
    };

    using BezierConfigPtr      = std::shared_ptr<BezierConfig>;
    using ConstBezierConfigPtr = std::shared_ptr<const BezierConfig>;
  }
}
