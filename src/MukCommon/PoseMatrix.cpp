#include "private/muk.pch"
#include "PoseMatrix.h"

#include <Eigen/Geometry>

namespace gris
{
namespace se3
{
  /**
  */
  void PoseMatrix::setPosition(const Vec3d& p)
  { 
    for (size_t i(0); i < 3; ++i) 
      mData(i, 3) = p[i]; 
  }

  /**
  */
  void PoseMatrix::setOrientation(const Quaternion& p)
  {
    const auto R = Eigen::Quaterniond(p.w(), p.x(), p.y(), p.z()).toRotationMatrix();
    for (size_t i(0); i < 3; ++i)
      for (size_t j(0); j < 3; ++j)
        mData(i, j) = R(i,j);
  }

  /**
  */
  Vec3d PoseMatrix::getPosition() const
  {
    Vec3d p(mData(0,3), mData(1,3), mData(2,3));
    return p;
  }

  /**
  */
  PoseMatrix PoseMatrix::operator*(const PoseMatrix& o) const
  {
    return PoseMatrix(mData*o.data());
  }

  /** \brief The log operator on SE(3)

    see Murray, Li, Systry, 1994: A Mathematical Introduction to Robotic Manipulation, Appendix A

    \input X: an element of SE(3)
  */
  TwistMatrix PoseMatrix::log() const
  {
    const auto& X = mData;
    Eigen::Matrix3d R;
    for (size_t i(0); i<3; ++i)
      for (size_t j(0); j<3; ++j)
        R(i, j) = X(i, j);

    // first compute the log operator on SO(3)
    Eigen::Matrix3d w_hat;
    w_hat.fill(0.0);
    double theta(0);
    // if R == Identity is represented by the initial values of w and theta
    if ( ! R.isIdentity(1e-8))
    {
      theta = std::acos( (R.trace() - 1.0) / 2.0 );
      w_hat = 1.0 / (2 * std::sin(theta)) * (R - R.transpose());
    }
    // then proceed for SE(3)
    const auto w  = Vec3d(w_hat(2, 1), w_hat(0, 2), w_hat(1, 0));
    const auto wn = w.norm();
    const auto sinwn = std::sin(wn);
    const auto coswn = std::cos(wn);
    const auto w_hat_squared = w_hat*w_hat;
    const auto fraction = (2 * sinwn - wn*(1 + coswn)) / (2 * wn*wn * sinwn);
    const auto A_inverse = Eigen::Matrix3d::Identity() - 1.0/2.0 * w_hat +  fraction*w_hat_squared;
    const auto p     = Eigen::Vector3d(X(0, 3), X(1, 3), X(2, 3));
    const auto p_inv = A_inverse * p;
    TwistMatrix U;
    for (int i(0); i<3; ++i)
    {
      for (int j(0); j<3; ++j)
        U(i, j) = w_hat(i, j);
      U(i, 3) = p_inv[i];
    }
    return U;
  }
}
}