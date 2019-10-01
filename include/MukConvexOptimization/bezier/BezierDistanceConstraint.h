#pragma once
#include "bezier/BezierOptimizationModel.h"

#include "opt/Constraint.h"

namespace gris
{
  namespace bezier
  {
    /** \brief A simple quadratic cost function on trajectories, adding up their distance to risk structures
    */
    class BezierDistanceConstraint : public opt::IneqConstraint
    {
    public:
      BezierDistanceConstraint(const BezierOptimizationModel& model, const muk::ICollisionDetector* pColl);

    public:
      void setDistanceThreshold(double d) { mDistanceThreshold = d;  }

    public:
      virtual opt::ConvexConstraintsPtr   convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const;
      virtual std::vector<double>         value(const std::vector<double>& x) const;

    private:
      const muk::ICollisionDetector* mpColl;
      const BezierOptimizationModel& mModel;
      double mWeight;
      double mDistanceThreshold;
    };
  }
}