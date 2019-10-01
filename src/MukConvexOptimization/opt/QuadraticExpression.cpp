#include "opt/QuadraticExpression.h"

#include <iostream>

namespace gris
{
namespace opt
{
  /**
  */
  double QuadraticExpression::value(const std::vector<double>& x) const
  {
    double out = mAffineExpression.value(x);
    for (size_t i=0; i < size(); ++i)
      out += mCoeffs[i] * mVars1[i].value(x) * mVars2[i].value(x);
    return out;
  }

  /**
  */
  double QuadraticExpression::value(const double* x) const
  {
    double out = mAffineExpression.value(x);
    for (size_t i=0; i < size(); ++i)
      out += mCoeffs[i] * mVars1[i].value(x) * mVars2[i].value(x);
    return out;
  }

  /**
  */
  QuadraticExpression QuadraticExpression::square(const Variable& a)
  {
    QuadraticExpression out;
    out.coeffs().push_back(1);
    out.vars1().push_back(a);
    out.vars2().push_back(a);
    return out;
  }

  /**
  */
  QuadraticExpression QuadraticExpression::square(const AffineExpression& input)
  {
    QuadraticExpression out;
    size_t naff = input.coeffs().size();
    size_t nquad = (naff*(naff+1))/2;

    out.affine().constant() = std::pow(input.constant(), 2);

    out.affine().vars() = input.vars();
    out.affine().coeffs().resize(naff);
    for (size_t i = 0; i < naff; ++i)
      out.affine().coeffs()[i] = 2*input.constant()*input.coeffs()[i];

    out.coeffs().reserve(nquad);
    out.vars1().reserve(nquad);
    out.vars2().reserve(nquad);
    for (size_t i = 0; i < naff; ++i)
    {
      out.vars1().push_back(input.vars()[i]);
      out.vars2().push_back(input.vars()[i]);
      out.coeffs().push_back( std::pow(input.coeffs()[i], 2) );
      for (size_t j = i+1; j < naff; ++j)
      {
        out.vars1().push_back(input.vars()[i]);
        out.vars2().push_back(input.vars()[j]);
        out.coeffs().push_back(2 * input.coeffs()[i] * input.coeffs()[j]);
      }
    }
    return out;
  }

  /**
  */
  QuadraticExpression operator*(const AffineExpression& lhs, const AffineExpression& rhs)
  {
    QuadraticExpression out;
    // constant
    out.affine().constant() = lhs.constant() * rhs.constant();
    // each affine variable times the other's constant
    auto& aff = out.affine();
    for (size_t i(0); i<lhs.vars().size(); ++i)
    {
      aff.vars().push_back( lhs.vars()[i] );
      aff.coeffs().push_back( rhs.constant() * lhs.coeffs()[i] );
    }
    for (size_t i(0); i<rhs.vars().size(); ++i)
    {
      aff.vars().push_back( rhs.vars()[i] );
      aff.coeffs().push_back( lhs.constant() * rhs.coeffs()[i] );
    }
    // each lhs variable times all rhs vars
    for (size_t i(0); i<lhs.vars().size(); ++i)
    {
      for (size_t j(0); j<rhs.vars().size(); ++j)
      {
        out.vars1().push_back(lhs.vars()[i]);
        out.vars2().push_back(rhs.vars()[j]);
        out.coeffs().push_back(lhs.coeffs()[i] * rhs.coeffs()[j]);
      }
    }
    return out;
  }

  /**
  */
  void QuadraticExpression::make_stable(double epsilon)
  {
    QuadraticExpression out;
    out.affine().make_stable();
    for (size_t i = 0; i < size(); ++i)
    {
      if (fabs(mCoeffs[i]) > epsilon)
      {
        out.coeffs().push_back(mCoeffs[i]);
        out.vars1().push_back(mVars1[i]);
        out.vars2().push_back(mVars2[i]);
      }
    }
    *this = out;
  }


  /**
  */
  std::ostream& QuadraticExpression::operator<<(std::ostream& os) const
  {
    os << mAffineExpression;
    for (size_t i=0; i < size(); ++i)
    {
      os << " + " << mCoeffs[i] << "*" << mVars1[i] << "*" << mVars2[i];
    }
    return os;
  }
}
}