#pragma once
#include "bezier/BezierOptimizationModel.h"

#include "opt/Constraint.h"

namespace gris
{
  namespace bezier
  {
    /** \brief A simple quadratic cost function on trajectories, adding up their distance to risk structures
    */
    class BezierSplineDistanceConstraint : public opt::IneqConstraint
    {
    public:
      BezierSplineDistanceConstraint(const BezierOptimizationModel& model, const muk::ICollisionDetector* pColl);

    public:
      void setDistanceThreshold(double d) { mDistanceThreshold = d;  }
      void setBezierSplineIndex(size_t i);

    public:
      virtual opt::ConvexConstraintsPtr   convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const;
      virtual std::vector<double>         value(const std::vector<double>& x) const;

    private:
      const muk::ICollisionDetector* mpColl;
      const BezierOptimizationModel& mModel;
      double mDistanceThreshold;
      size_t mIdx;
    };
  }
}