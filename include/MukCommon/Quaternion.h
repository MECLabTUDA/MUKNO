#pragma once
#include "muk_common_api.h"

#include "gstd/Vector.h"

#include <array>

namespace gris
{
  /**
  */
  class MUK_COMMON_API Quaternion
  {
    public:
      static double QuaternionPrecision;

    public:
      Quaternion()                                  { mCoeff.fill(0); }
      explicit Quaternion(double* d)                { for (size_t i(0); i<mCoeff.size(); ++i) mCoeff[i] = d[i]; }
      explicit Quaternion(const Vec3d& p)           { w()=0, x()=p.x(); y()=p.y(); z()=p.z();}
      template<typename Q>
      Quaternion(Q w_, Q x_, Q y_, Q z_)            { w() = w_; x() = x_, y() = y_; z() = z_; }

    public:
      static Quaternion Identity();
      static Quaternion FromEulerVector(const Vec3d& v);
      static Quaternion FromAxisAngle(const Vec3d& v, double angle);
      
    public:
      double& x()             { return mCoeff[0]; }
      const double& x() const { return mCoeff[0]; }
      double& y()             { return mCoeff[1]; }
      const double& y() const { return mCoeff[1]; }
      double& z()             { return mCoeff[2]; }
      const double& z() const { return mCoeff[2]; }

      double& w()             { return mCoeff[3]; }
      const double& w() const { return mCoeff[3]; }

      double*       data()        { return mCoeff.data(); }
      const double* data() const  { return mCoeff.data(); }

    public:
      template<typename Scalar>
      Quaternion& operator*=(const Scalar& val)
      {
        for(size_t i(0); i<mCoeff.size(); ++i)
          mCoeff[i] *= val;
        return *this;
      }

      template<typename Scalar, typename = std::enable_if_t<std::is_fundamental<Scalar>::value>>
      friend Quaternion operator*(Quaternion q, Scalar val)
      {  
        q*=val;
        return q;
      }

      template<typename Scalar, typename = std::enable_if_t<std::is_fundamental<Scalar>::value>>
      friend Quaternion operator*(Scalar val, Quaternion q)
      {  
        q*=val;
        return q;
      }

      template<typename Scalar, typename = std::enable_if_t<std::is_fundamental<Scalar>::value>>
      friend Quaternion operator/(Quaternion q, Scalar val)
      {  
        q*=1.0/val;
        return q;
      }

      Quaternion& operator*=(const Quaternion& rhs)
      {
        *this = (*this) * rhs;
        return *this;
      }

      friend Vec3d operator*(Quaternion q, const Vec3d& v)
      {
        const auto u = Vec3d(q.x(),q.y(),q.z());
        auto uv = 2*u.cross(v);
        return v + q.w() * uv + u.cross(uv);
      }

      friend Quaternion operator*(const Quaternion& a, const Quaternion& b);

    public:
      double  dot(const Quaternion& q) const;
      double  norm()           const  { return std::sqrt(squaredNorm()); }
      double  squaredNorm()    const  { return dot(*this); }
      void    setIdentity()           { x() = y() = z() = 0; w() = 1; }
      void    normalize();
      Quaternion conjugate()   const  { return Quaternion(w(), -x(), -y(), -z()); }
      Quaternion inverse()     const;
      Quaternion normalized()  const { Quaternion r(*this); r.normalize(); return r; }
      Vec3d   toEulerVector()  const;
      Quaternion slerp(double t, const Quaternion& target) const;

    public:
      std::ostream& operator<<(std::ostream& os) const
      {
        os << w() << " ";
        os << x() << " ";
        os << y() << " ";
        os << z();
        return os;
      }

      std::istream& operator>>(std::istream& is)
      {
        is >> w();
        for (size_t i(0); i<mCoeff.size()-1; ++i)
        {
          // this is for boost::lexical_cast which does not ignore white spaces
          // see https://stackoverflow.com/questions/10382884/c-using-classes-with-boostlexical-cast
          if ((is.flags() & std::ios_base::skipws) == 0) 
          {
            char whitespace;
            is >> whitespace;
          }
          is >> mCoeff[i];
        }
        return is;
      }

      friend std::ostream& operator<<(std::ostream& os, const Quaternion& v) { return v.operator<<(os); } 
      friend std::istream& operator>>(std::istream& is,       Quaternion& v) { return v.operator>>(is); }

    private:
      std::array<double, 4> mCoeff;
  };

  /**
  */
  inline Quaternion operator*(const Quaternion& a, const Quaternion& b)
  {
    /*return Quaternion
      (
        a.w() * b.w()   -   a.x() * b.x()   -   a.y() * b.y()   -   a.z() * b.z(),
        a.w() * b.x()   +   a.x() * b.w()   +   a.y() * b.z()   -   a.z() * b.y(),
        a.w() * b.y()   -   a.x() * b.z()   +   a.y() * b.w()   +   a.z() * b.x(),
        a.w() * b.z()   +   a.x() * b.y()   -   a.y() * b.x()   +   a.z() * b.w()
      );                                                     */
    return Quaternion(
       a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
       a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
       a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
       a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x());
  }
}
