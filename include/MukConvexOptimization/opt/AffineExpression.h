#pragma once
#include "opt/Variable.h"

#include <algorithm>
#include <iterator>
#include <vector>

namespace gris
{
  namespace opt
  {
    /** \brief Affine expression

      f(x) = constant + coeffs[0] * vars[0].value(x) + ... + coeffs[n] * vars[n].value(x);
    */
    class AffineExpression 
    { 
      public:
        AffineExpression()                             : mConstant(0) {}
        explicit AffineExpression(double a)            : mConstant(a) {}
        explicit AffineExpression(const Variable& v)   : mConstant(0), mCoeffs(1, 1), mVariables(1, v) {}

      public:
        void                    make_stable(double epsilon = 1e-7);
        double&                 constant()            { return mConstant; }
        std::vector<double>&    coeffs()              { return mCoeffs; }
        std::vector<Variable>&  vars()                { return mVariables; }
        const double&           constant()      const { return mConstant; }
        const std::vector<double>&    coeffs()  const { return mCoeffs; }
        const std::vector<Variable>&  vars()    const { return mVariables; }

      public:
        size_t size()                              const {return mCoeffs.size();}
        double value(const double* x)              const;
        double value(const std::vector<double>& x) const;

      public:
        std::ostream& operator<<(std::ostream& os) const;
        friend std::ostream& operator<<(std::ostream& os, const AffineExpression& v) { return v.operator<<(os); }


      public:
        template<class Scalar>
        AffineExpression& operator*=(Scalar d)
        {
          mConstant *= d;
          for (size_t i = 0; i < mCoeffs.size(); ++i)
            mCoeffs[i] *= d;
          return *this;
        }

        template<class Scalar>
        friend AffineExpression operator*(AffineExpression lhs, Scalar d)
        {
          lhs *= d;
          return lhs;
        }

        template<class Scalar>
        friend AffineExpression operator*(Scalar d, AffineExpression rhs)
        {
          rhs *= d;
          return rhs;
        }

        template<class Scalar>
        AffineExpression& operator+=(Scalar d)
        {
          mConstant += d;
          return *this;
        }

        template<class Scalar>
        friend AffineExpression operator+(AffineExpression lhs, Scalar d)
        {
          lhs += d;
          return lhs;
        }

        template<class Scalar>
        friend AffineExpression operator+(Scalar d, AffineExpression lhs)
        {
          lhs += d;
          return lhs;
        }

        template<class Scalar>
        AffineExpression& operator-=(Scalar d)
        {
          mConstant -= d;
          return *this;
        }

        template<class Scalar>
        friend AffineExpression operator-(AffineExpression lhs, Scalar d)
        {
          lhs -= d;
          return lhs;
        }

        AffineExpression operator-() const
        {
          AffineExpression ret (*this);
          ret *= (-1);
          return ret;
        }

        AffineExpression& operator+=(const AffineExpression& rhs)
        {
          mConstant += rhs.mConstant;
          std::transform(rhs.mCoeffs.begin(), rhs.mCoeffs.end(), std::back_inserter(mCoeffs), [&] (const auto& d) { return d; });
          std::copy(rhs.mVariables.begin(), rhs.mVariables.end(), std::back_inserter(mVariables));
          return *this;
        }

        friend AffineExpression operator+(AffineExpression lhs, const AffineExpression& rhs)
        {
          lhs += rhs;
          return lhs;
        }

        AffineExpression& operator-=(const AffineExpression& rhs)
        {
          mConstant -= rhs.mConstant;
          std::transform(rhs.mCoeffs.begin(), rhs.mCoeffs.end(), std::back_inserter(mCoeffs), [&] (const auto& d) { return -d; });
          std::copy(rhs.mVariables.begin(), rhs.mVariables.end(), std::back_inserter(mVariables));
          return *this;
        }

        friend AffineExpression operator-(AffineExpression lhs, const AffineExpression& rhs)
        {
          lhs -= rhs;
          return lhs;
        }

        AffineExpression& operator+=(const Variable& rhs)
        {
          *this += AffineExpression(rhs);
          return *this;
        }

        friend AffineExpression operator+(AffineExpression lhs, const Variable& rhs)
        {
          lhs += rhs;
          return lhs;
        }

        AffineExpression& operator-=(const Variable& rhs)
        {
          *this -= AffineExpression(rhs);
          return *this;
        }

        friend AffineExpression operator-(AffineExpression lhs, const Variable& rhs)
        {
          lhs -= rhs;
          return lhs;
        }

      private:
        double mConstant;
        std::vector<double> mCoeffs;
        std::vector<Variable> mVariables;
    };
  }
}