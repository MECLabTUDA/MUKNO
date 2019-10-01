#pragma once
#include "bezier/BezierOptimizationModel.h"

#include "opt/Cost.h"

namespace gris
{
  namespace bezier
  {
    /** \brief A simple quadratic cost function on trajectories, adding up their linearly interpolated length
    */
    class BezierLengthCost : public opt::Cost
    {
      public:
        BezierLengthCost(const BezierOptimizationModel& model, const muk::VariableMatrix& positionVariables, const std::vector<ConstBezierConfigPtr>& configs, double weight);

      public:
        virtual opt::ConvexObjectivePtr convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const;
        virtual double value(const std::vector<double>& x) const;

      private:
        opt::QuadraticExpression mExpression;
        const std::vector<ConstBezierConfigPtr> mConfigs;
        const BezierOptimizationModel& mModel;
        const muk::VariableMatrix mVariables;
        double mWeight;
    };
  }
}
