#include "opt/AffineExpression.h"

#include <iostream>

namespace gris
{
namespace opt
{
  /**
  */
  double AffineExpression::value(const double* x) const
  {
    double out = mConstant;
    for (size_t i=0; i < size(); ++i)
      out += mCoeffs[i] * mVariables[i].value(x);
    return out;
  }

  /**
  */
  double AffineExpression::value(const std::vector<double>& x) const 
  {
    double out = mConstant;
    for (size_t i=0; i < size(); ++i)
      out += mCoeffs[i] * mVariables[i].value(x);
    return out;
  }

  /** \brief Creates an Affine Expression that has no coefficients smaller than epsilon

    \param[i] epsilon threshold that a coefficient has to exceed
    \return   A copy of the expression with no coefficients smaller than the given threshold #epsilon
  */
  void AffineExpression::make_stable(double epsilon)
  {
    AffineExpression out;
    for (size_t i = 0; i < size(); ++i)
    {
      if (std::fabs(mCoeffs[i]) > epsilon)
      {
        out.coeffs().push_back(mCoeffs[i]);
        out.vars().push_back(mVariables[i]);
      }
    }
    out.constant() = mConstant;
    *this = out;
  }

  /**
  */
  std::ostream& AffineExpression::operator<<(std::ostream& os) const
  {
    os << mConstant;
    for (size_t i=0; i < size(); ++i) 
    {
      os << " + " << mCoeffs[i] << "*" << mVariables[i];
    }
    return os;
  }
}
}