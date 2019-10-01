#include "opt/FunctionConstraint.h"

#include <iostream>

namespace
{
  using namespace gris;

  /**
  */
  Eigen::VectorXd toEigenVector(const std::vector<double>& x, const std::vector<opt::Variable>& vars)
  {
    Eigen::VectorXd out(vars.size());
    for (size_t i=0; i < vars.size(); ++i) 
      out[i] = x[vars[i].getRep()->getIndex()];
    return out;
  }

  /**
  */
  inline std::vector<double> toStdVector(const Eigen::VectorXd& x)
  {
    return std::vector<double>(x.data(), x.data()+x.size());
  }

  /**
  */
  opt::AffineExpression fromValGrad(double y, const Eigen::VectorXd& x, const Eigen::VectorXd& dydx, const std::vector<opt::Variable>& vars);
}

namespace gris
{
namespace opt
{
  /**
  */
  FunctionConstraint::FunctionConstraint(std::shared_ptr<VectorToVectorFunction> f, const std::vector<Variable>& vars, const Eigen::VectorXd& weights, ConstraintType type, const std::string& name)
    : Constraint(name)
    , mFunction(f)
    , mDerivative(nullptr)
    , mVars(vars)
    , mWeights(weights)
    , mType(type)
    , mEpsilon(1e-5)
  {
  }

  /**
  */
  std::vector<double> FunctionConstraint::value(const std::vector<double>& xin) const
  {
    const auto x = toEigenVector(xin, mVars);
    auto err = mFunction->call(x);
    if (mWeights.size()>0)
      err.array() *= mWeights.array(); // component wise multiplication = weighting
    return toStdVector(err);
  }

  /**
  */
  ConvexConstraintsPtr FunctionConstraint::convexify(const std::vector<double>& xin, ConvexOptimizationSolver& model) const
  {
    const auto x   = toEigenVector(xin, mVars);
    const auto jac = mDerivative != nullptr ? mDerivative->call(x) : forwardNumericalJacobian(*mFunction, x);
    auto out       = std::make_shared<ConvexConstraints>(&model);
    const auto y   = mFunction->call(x);

    for (int i=0; i < jac.rows(); ++i) 
    {
      auto aff = fromValGrad(y[i], x, jac.row(i), mVars);
      if (mWeights.size() > 0) 
      {
        if (mWeights[i] == 0)
          continue;
        aff *= mWeights[i];
      }
      if (type() == enINEQ) 
        out->addIneqCnt(aff);
      else 
        out->addEqCnt(aff);
    }
    return out;
  }

  /**
  */
  Eigen::MatrixXd FunctionConstraint::forwardNumericalJacobian(const VectorToVectorFunction& f, const Eigen::VectorXd& x) const
  {
    using namespace Eigen;
    VectorXd y = f(x);
    MatrixXd out(y.size(), x.size());
    VectorXd x_perturbed = x;
    for (size_t i=0; i < size_t(x.size()); ++i) 
    {
      x_perturbed(i) = x(i) + mEpsilon;
      VectorXd ypert = f(x_perturbed);
      out.col(i) = (ypert - y) / mEpsilon;
      x_perturbed(i) = x(i);
    }
    return out;
  }
}
}

namespace
{
  /**
  */
  opt::AffineExpression fromValGrad(double y, const Eigen::VectorXd& x, const Eigen::VectorXd& dydx, const std::vector<opt::Variable>& vars)
  {
    opt::AffineExpression aff;
    aff.constant() = y - dydx.dot(x);
    aff.coeffs() = toStdVector(dydx);
    aff.vars() = vars;
    aff.make_stable();
    return aff;
  }
}