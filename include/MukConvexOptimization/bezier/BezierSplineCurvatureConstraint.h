#pragma once
#include "bezier/BezierOptimizationModel.h"

#include "opt/Constraint.h"

namespace gris
{
  namespace bezier
  {
    /** \brief A simple quadratic cost function on trajectories, adding up their distance to risk structures
    */
    class BezierSplineCurvatureConstraint : public opt::IneqConstraint
    {
    public:
      BezierSplineCurvatureConstraint(const BezierOptimizationModel& model, size_t idx);

    public:
      virtual opt::ConvexConstraintsPtr   convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const;
      virtual std::vector<double>         value(const std::vector<double>& x) const;

    private:
      const BezierOptimizationModel&  mModel;
      size_t mIdx;                                // the index of a waypoint triple (W_i, W_i+1, W_i+2))
    };
  }
}