#pragma once
#include "AffineExpression.h"

#include <vector>

namespace gris
{
  namespace opt
  {
    /** Quadratic Expression

      f(x) = constant + coeffs[0] * vars[0].value(x)^2 + ... + coeffs[n] * vars[n].value(x)^2;
    */
    class QuadraticExpression
    {
      public:
        QuadraticExpression() {}
        explicit QuadraticExpression(double a)                    : mAffineExpression(a) {}
        explicit QuadraticExpression(const Variable& v)           : mAffineExpression(v) {}
        explicit QuadraticExpression(const AffineExpression& aff) : mAffineExpression(aff) {}

      public:
        static QuadraticExpression square(const Variable& a);
        static QuadraticExpression square(const AffineExpression& a);

      public:
        size_t size()                               const { return mCoeffs.size(); }
        double value(const double* x)               const;
        double value(const std::vector<double>& x)  const;

      public:
        void                    make_stable(double epsilon = 1e-8);
        AffineExpression&       affine()            { return mAffineExpression; }
        std::vector<double>&    coeffs()            { return mCoeffs; }
        std::vector<Variable>&  vars1()             { return mVars1; }
        std::vector<Variable>&  vars2()             { return mVars2; }
        const AffineExpression& affine()     const  { return mAffineExpression; }
        const std::vector<double>& coeffs()  const  { return mCoeffs; }
        const std::vector<Variable>& vars1() const  { return mVars1; }
        const std::vector<Variable>& vars2() const  { return mVars2; }

      public:
        template<class Scalar>
        QuadraticExpression& operator*=(Scalar d)
        {
          mAffineExpression *= d;
          for (size_t i = 0; i < mCoeffs.size(); ++i)
            mCoeffs[i] *= d;
          return *this;
        }

        template<class Scalar>
        friend QuadraticExpression operator*(QuadraticExpression lhs, Scalar d)
        {
          lhs *= d;
          return lhs;
        }

        template<class Scalar>
        friend QuadraticExpression operator*(Scalar d, QuadraticExpression rhs)
        {
          rhs *= d;
          return rhs;
        }

        template<class Scalar>
        QuadraticExpression& operator+=(Scalar d)
        {
          mAffineExpression += d;
          return *this;
        }

        template<class Scalar>
        friend QuadraticExpression operator+(QuadraticExpression lhs, Scalar d)
        {
          lhs += d;
          return lhs;
        }

        template<class Scalar>
        friend QuadraticExpression operator+(Scalar d, QuadraticExpression lhs)
        {
          lhs += d;
          return lhs;
        }

        QuadraticExpression& operator+=(const AffineExpression& rhs)
        {
          mAffineExpression += rhs;
          return *this;
        }

        friend QuadraticExpression operator+(QuadraticExpression lhs, const AffineExpression& rhs)
        {
          lhs += rhs;
          return lhs;
        }

        friend QuadraticExpression operator+(const AffineExpression& lhs, QuadraticExpression rhs)
        {
          rhs += lhs;
          return rhs;
        }

        QuadraticExpression& operator+=(QuadraticExpression rhs)
        {
          mAffineExpression += rhs.affine();
          coeffs().insert(coeffs().end(), rhs.coeffs().begin(), rhs.coeffs().end());
          vars1() .insert(vars1().end(),  rhs.vars1().begin(),  rhs.vars1().end());
          vars2() .insert(vars2().end(),  rhs.vars2().begin(),  rhs.vars2().end());
          return *this;
        }

        friend QuadraticExpression operator+(QuadraticExpression lhs, const QuadraticExpression& rhs)
        {
          lhs += rhs;
          return lhs;
        }

        template<class Scalar>
        QuadraticExpression& operator-=(Scalar d)
        {
          mAffineExpression -= d;
          return *this;
        }

        template<class Scalar>
        friend QuadraticExpression operator-(QuadraticExpression lhs, Scalar d)
        {
          lhs -= d;
          return lhs;
        }

        template<class Scalar>
        friend QuadraticExpression operator-(Scalar d, QuadraticExpression lhs)
        {
          lhs -= d;
          return lhs;
        }

        QuadraticExpression& operator-=(AffineExpression rhs)
        {
          mAffineExpression -= rhs;
          return *this;
        }

        friend QuadraticExpression operator-(QuadraticExpression lhs, const AffineExpression& rhs)
        {
          lhs -= rhs;
          return lhs;
        }

        friend QuadraticExpression operator-(const AffineExpression& lhs, QuadraticExpression rhs)
        {
          rhs -= lhs;
          return rhs;
        }

        QuadraticExpression& operator-=(QuadraticExpression rhs)
        {
          *this *= -1;
          *this += rhs;
          return *this;
        }

        friend QuadraticExpression operator-(QuadraticExpression lhs, const QuadraticExpression& rhs)
        {
          lhs -= rhs;
          return lhs;
        }

        friend QuadraticExpression operator*(const AffineExpression& lhs, const AffineExpression& rhs);

      public:
        std::ostream& operator<<(std::ostream& os) const;
        friend std::ostream& operator<<(std::ostream& os, const QuadraticExpression& v) { return v.operator<<(os); }
  
      private:
        AffineExpression mAffineExpression;
        std::vector<double> mCoeffs;
        std::vector<Variable> mVars1;
        std::vector<Variable> mVars2;
    };
  }
}