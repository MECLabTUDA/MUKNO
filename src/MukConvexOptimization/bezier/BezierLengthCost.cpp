#include "private/muk.pch"
#include "bezier/BezierLengthCost.h"

namespace gris
{
namespace bezier
{
  /**
  */
  BezierLengthCost::BezierLengthCost(const BezierOptimizationModel& model, const muk::VariableMatrix& positionVariables, const std::vector<ConstBezierConfigPtr>& configs, double weight)
    : mVariables(positionVariables)
    , mConfigs(configs)
    , mModel(model)
    , mWeight(weight)
  {
    mName = "Length";
    // already create the convexified expression of this cost term here.
    for (size_t i=1; i < mVariables.rows(); ++i) 
    {
      for (size_t j=0; j < mVariables.cols(); ++j) 
      {
        auto vel = opt::AffineExpression(mVariables(i-1,j)) - opt::AffineExpression(mVariables(i,j));
        mExpression += mWeight * opt::QuadraticExpression::square(vel);
      }
    }
  }

  /**
  */
  opt::ConvexObjectivePtr BezierLengthCost::convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const
  {
    auto out = std::make_shared<opt::ConvexObjective>(&model);
    out->addQuadraticExpression(mExpression);
    return out;
  }

  /**
  */
  double BezierLengthCost::value(const std::vector<double>& x) const
  {
    double val(0);
    for (size_t i=1; i < mVariables.rows(); ++i) 
      for (size_t j=0; j < mVariables.cols(); ++j) 
      {
        auto diff = mVariables(i-1,j).value(x) - mVariables(i,j).value(x);
        val += mWeight * diff * diff;
      }
    return val;
  }
}
}