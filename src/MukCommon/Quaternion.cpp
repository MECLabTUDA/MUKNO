#include "private/muk.pch"
#include "Quaternion.h"

namespace gris
{
  /**
  */
  double Quaternion::QuaternionPrecision = 1e-10;

  /** \brief Returns a quaternion from an angle axis representation
  
    \param v vector with "direction = rotation axis" and "length = rotation angle in radians"
  */
  Quaternion Quaternion::FromEulerVector(const Vec3d& v)
  {
    /*auto l2 = v.squaredNorm();
    if( l2 == 0 )
      return Quaternion(1.0, 0.0, 0.0, 0.0);
    const auto len = std::sqrt(l2);
    const auto angle_2 = 0.5* len;
    const auto directionCosines  = (std::sin(angle_2) / len) * v;
    return Quaternion(std::cos(angle_2), directionCosines.x(), directionCosines.y(), directionCosines.z());*/
    auto axislen = std::sqrt(v.squaredNorm());
    if( axislen == 0 ) {
      return Quaternion(1,0,0,0);
    }
    const auto sang = std::sin(axislen*0.5) / axislen;
    return Quaternion(std::cos(axislen*0.5), v.x()*sang, v.y()*sang, v.z()*sang);
  }

  /**
  */
  Quaternion Quaternion::FromAxisAngle(const Vec3d& v, double angle)
  {
    /*angle *= 0.5;
    const auto directionCosines  = std::sin(angle) * v;
    return Quaternion(std::cos(angle), directionCosines.x(), directionCosines.y(), directionCosines.z());*/
    auto axislen = v.norm();
    if( axislen == 0 ) 
      return Quaternion(1,0,0,0);
    angle *= 0.5;
    const auto sang = std::sin(angle) / axislen;
    return Quaternion(std::cos(angle), v.x()*sang, v.y()*sang, v.z()*sang);
  }

  /**

    it is faster to multiply by (1/f), but with 4 divides we gain precision (which is more important in robotics)
  */
  void Quaternion::normalize() 
  {
    auto l2 = squaredNorm(); 
    using Lim = std::numeric_limits<double>;

    if(  l2 < 1.0 - Lim::epsilon() || l2 > 1.0 + Lim::epsilon() ) 
    {
      l2 = std::sqrt(l2);
      for (size_t i(0); i<mCoeff.size(); ++i)
        mCoeff[i] /= l2;
    }
  }

  /**
  */
  Quaternion Quaternion::inverse() const 
  {
    const auto n2 = this->squaredNorm();
    if (n2 > 0.0)
    {
      return conjugate() / n2;
    }
    else
    {
      // return an invalid result to flag the error
      return Quaternion(0.0, 0.0, 0.0, 0.0);
    }
  }

  /** \brief A Quaternion with w=1, x=y=z=0.
  */
  Quaternion Quaternion::Identity()
  {
    Quaternion q; 
    q.setIdentity(); 
    return q;
  }

  /** \brief returns an Euler vector that represents the same rotation

    Requires a normalized quaternion.
  */
  Vec3d Quaternion::toEulerVector() const
  {
    Vec3d res;
    const auto angle = 2 * std::acos(w());
    const double s = std::sqrt(1-w()*w()); // assuming quaternion normalised then w is less than 1, so term always positive.
    if (s < QuaternionPrecision) 
    {
      res.x() = x(); // if it is important that axis is normalised then replace with, e.g., x=1; y=z=0;
      res.y() = y();
      res.z() = z();
    }
    else 
    {
      res.x() = x() / s; // normalise axis
      res.y() = y() / s;
      res.z() = z() / s;
      res *= angle;
    }
    return res;
  }

  /** \brief Dot product for Quaternions is just the the Euclidean dot product
  */
  double Quaternion::dot(const Quaternion& q) const
  {
    double val(0);
    for (size_t i(0); i < mCoeff.size(); ++i)
      val += mCoeff[i] * q.mCoeff[i];
    return val;
  }

  /** \brief Spherical linear Interpolation (Slerp) from *this to target. 
  
    Quaternion and target have to be normalized
    \param t:      time in [0,1]
    \param target: normalized quaternion
  */
  Quaternion Quaternion::slerp(double t, const Quaternion& target) const
  {
    static const double eps = 1.0 - std::numeric_limits<double>::epsilon();
    const auto d    = dot(target);
    const auto dabs = std::abs(d);

    double scale0;
    double scale1;

    if(dabs >= eps)
    {
      scale0 = 1.0 - t;
      scale1 = t;
    }
    else
    {
      // theta is the angle between the 2 quaternions
      double theta  = acos(dabs);
      double stheta = std::sin(theta);
      scale0 = sin( ( 1.0 - t ) * theta) / stheta;
      scale1 = sin( ( t * theta) ) / stheta;
    }
    if(d<0)
      scale1 = -scale1;

    auto res = Quaternion();
    for (size_t i(0); i < mCoeff.size(); ++i)
      res.mCoeff[i] = scale0 * mCoeff[i] + scale1 * target.mCoeff[i];
    return res;
  }
}