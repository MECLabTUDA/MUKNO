#include "private/muk.pch"
#include "TwistMatrix.h"

#include <Eigen/Geometry>

namespace
{
  Eigen::Matrix4d toEigen(const gris::Matrix4d& M)
  {
    Eigen::Matrix4d ret;
    for (size_t i(0); i < M.rows(); ++i)
      for (size_t j(0); j < M.cols(); ++j)
        ret(i, j) = M(i, j);
    return ret;
  }

  gris::Matrix4d toGris(const Eigen::Matrix4d& M)
  {
    gris::Matrix4d ret;
    for (int i(0); i < M.rows(); ++i)
      for (int j(0); j < M.cols(); ++j)
        ret(i, j) = M(i, j);
    return ret;
  }
}

namespace gris
{
namespace se3
{
  /**
  */
  TwistMatrix::TwistMatrix(const PoseMatrix& from, const PoseMatrix& to)
  {
    const auto inv = toGris(toEigen(from.data()).inverse());
    *this = Pose3D(inv*to.data()).log();
  }

  /** \brief The expontential operator on SE(3)

  see Murray, Li, Systry, 1994: A Mathematical Introduction to Robotic Manipulation, Appendix A

  \input U: an element of se(3)
  */
  PoseMatrix TwistMatrix::exp() const
  {
    const auto xi = vee();
    const auto v  = Vec3d(xi[0], xi[1], xi[2]);
    const auto w  = Vec3d(xi[3], xi[4], xi[5]);
    Matrix4d pose;
    if (w.isZero())
    {
      pose.setIdentity();
      for (size_t i(0); i<3; ++i)
        pose(i, 3) = v[i];
    }
    else
    {
      // compute exp of SO(3)
      // copy skew symmetric matrix
      Matrix3d w_hat;
      for (size_t i(0); i<3; ++i)
        for (size_t j(0); j<3; ++j)
          w_hat(i, j) = mData(i, j);
      const auto norm2 = w.squaredNorm();
      const auto norm  = std::sqrt(norm2);
      const auto cnorm = std::cos(norm);
      const auto snorm = std::sin(norm);
      const auto skewSquared = w_hat * w_hat;
      const auto exp_w = Matrix3d::Identity() + (snorm / norm) * w_hat + (1 - cnorm) / norm2 * skewSquared;

      const auto A  = Matrix3d::Identity() + (1 - cnorm) / norm2 * w_hat + (norm - snorm) / (norm*norm2) * skewSquared;
      const auto Av = A*v;
      // linear velocity
      for (size_t i(0); i < 3; ++i)
        pose(i, 3) = Av[i];
      // angular velocity
      for (size_t i(0); i < 3; ++i)
        for (size_t j(0); j < 3; ++j)
          pose(i, j) = exp_w(i,j);
      pose(3, 3) = 1;
    }
    return PoseMatrix(std::move(pose));
  }

  /**
  */
  PoseMatrix TwistMatrix::push(double t) const
  {
    return TwistMatrix(t*mData).exp();
  }

  /**
  */
  TwistVector TwistMatrix::vee() const
  {
    TwistVector xi;
    for (int i(0); i<3; ++i)
      xi[i] = mData(i, 3);
    xi[3] = mData(2, 1);
    xi[4] = mData(0, 2);
    xi[5] = mData(1, 0);
    return xi;
  }
}
}