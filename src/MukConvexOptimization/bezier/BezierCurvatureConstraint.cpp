#include "private/muk.pch"
#include "bezier/BezierCurvatureConstraint.h"

namespace gris
{
namespace bezier
{
  /**
  */
  BezierCurvatureConstraint::BezierCurvatureConstraint(const BezierOptimizationModel& model, size_t idx)
    : mModel(model)
    , mIdx(idx)
  {
    // fill the member vector of variables used by this class
    const auto w1 = mModel.getVariables(idx);
    const auto w2 = mModel.getVariables(idx+1);
    const auto w3 = mModel.getVariables(idx+2);
    for (const auto& var : w1)
      mVariables.push_back(var);
    for (const auto& var : w2)
      mVariables.push_back(var);
    for (const auto& var : w3)
      mVariables.push_back(var);
  }

  /** \brief Evaluates the curvature violation.

    Error indicates the distance to a violation free configuration.

    \param x: the current values of the variables used here, 9 dimensional
    \return a vector of the same size as the input vector, indicating a correction vector 
  */
  Eigen::VectorXd BezierCurvatureConstraint::operator() (const Eigen::VectorXd& x) const
  {
    std::array<Vec3d, 3> waypoints;
    for (int i(0); i<3; ++i)
    {
      waypoints[i] = Vec3d(x[3*i], x[3*i+1], x[3*i+2]);
    }
    const auto& w1 = waypoints[0];
    const auto& w2 = waypoints[1];
    const auto& w3 = waypoints[2];
    const auto kappa = mModel.getConfigs()[mIdx]->getBuilder().computeKappaMax(w1,w2,w3);
    auto result = Eigen::VectorXd();
    result.resize(x.size(), 1);
    result.fill(0.0);
    if (kappa < mModel.getKappaMax())
    {
      return result;
    }
    auto weight = 1 - mModel.getKappaMax() / kappa;
    auto isNan = [&](double d) { return !(d == d); };
    weight = isNan(weight) ? 0.01 : weight;
    //LOG_LINE << "  " << mIdx << ": " << kappa << " > " << mModel.getKappaMax();
    const auto P  = 0.5*(w1 + w3);
    const auto Q  = 0.5*(w2 + P);
    const auto w2New = P; // 0.5*(Q + w2);
    const auto v  = (P - w1).normalized();
    const auto d1 = (w1 - w2).norm();
    const auto d2 = (w3 - w2).norm();
    const auto w1New = P - 1.1*d1*v;
    const auto w3New = P + 1.1*d2*v;

    const auto diff1 = w1New - w1;
    const auto diff2 = w2New - w2;
    const auto diff3 = w3New - w3;
    
    if (mIdx == 0)
    {
      const auto w3New = w2 + d2*(w2 - w1).normalized();;
      //const auto diff3 = w3New - w3;
      const auto diff3 = w3-w3New;
      for (int i(0); i < 3; ++i)
        result[6+i] = diff3[i];
    }
    else if (mIdx == mMaxIdx)
    {
      const auto w1New = w2 + d1*(w2 - w3).normalized();;
      //const auto diff1 = w1New - w1;
      const auto diff1 = w1-w1New;
      for (int i(0); i < 3; ++i)
        result[i] = diff1[i];
    }
    else
    {
      for (int i(0); i < 3; ++i)
        result[i] = diff1[i];
      for (int i(0); i < 3; ++i)
        result[3+i] = diff2[i];
      for (int i(0); i < 3; ++i)
        result[6+i] = diff3[i];
      result *= weight;
    }
    //LOG_LINE << "diff " << result;
    return result;
  }
}
}