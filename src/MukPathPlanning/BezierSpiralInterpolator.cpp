#include "private/muk.pch"
#include "BezierSpiralInterpolator.h"
#include "BezierSpline.h"

#include "MukCommon/geometry.h"
#include "MukCommon/MukException.h"

#include <Eigen/Dense>

#include "MukCommon/gris_math.h"

namespace
{
  using namespace Eigen;
  using namespace gris::muk;
  using gris::Vec3d;
  using gris::M_Pi;

  double wideAngle(const Vec3d& v, const Vec3d& w)
  {
    auto T1 = v.normalized();
    auto T2 = w.normalized();
    return std::atan2(T1.cross(T2).norm(), T1.dot(T2));
  }

  template<typename MATRIX, typename VECTOR, size_t DIM>
  inline void fill_column(MATRIX& dest, size_t idx, const VECTOR& src)
  {
    for (size_t i(0); i<DIM; ++i)
      dest(i, idx) = src(i);
  }

  Matrix4d createTrafoMatrix(const Vector3d& column1, const Vector3d& column2, const Vector3d& column3, const Vector3d& column4)
  {
    Matrix4d result;
    fill_column<Matrix4d, Vector3d, 3>(result, 0, column1);
    fill_column<Matrix4d, Vector3d, 3>(result, 1, column2);
    fill_column<Matrix4d, Vector3d, 3>(result, 2, column3);
    fill_column<Matrix4d, Vector3d, 3>(result, 3, column4);
    result.row(3).fill(0);
    result(3, 3) = 1;
    return result;
  }

  MatrixXd createTrafoMatrix(const Vector2d& B0, const Vector2d& B1, const Vector2d& B2, const Vector2d& B3, const Vector2d& E0, const Vector2d& E1, const Vector2d& E2)
  {
    MatrixXd result = MatrixXd(4, 7);
    fill_column<MatrixXd, Vector2d, 2>(result, 0, B0);
    fill_column<MatrixXd, Vector2d, 2>(result, 1, B1);
    fill_column<MatrixXd, Vector2d, 2>(result, 2, B2);
    fill_column<MatrixXd, Vector2d, 2>(result, 3, B3);
    fill_column<MatrixXd, Vector2d, 2>(result, 4, E0);
    fill_column<MatrixXd, Vector2d, 2>(result, 5, E1);
    fill_column<MatrixXd, Vector2d, 2>(result, 6, E2);
    result.row(2).fill(0);
    result.row(3).fill(1);
    return result;
  }

  Vector2d project_3d_2d(const Matrix4d& M, const Vector3d& v)
  {
    Vector4d tmp = M*Vector4d(v.x(), v.y(), v.z(), 1);
    return Vector2d(tmp.x(), tmp.y());
  }

  struct Convert
  {
    gris::Vec3d operator() (const Vector3d& p)
    {
      return gris::Vec3d(p.data());
    }
    Vector3d operator() (const gris::Vec3d& p)
    {
      return  Vector3d(p.data());
    }
  };

  /** no reference!
  assume P_x = (0.5,0), P_e = (1, xxx)
  */
  double computeAlpha(const Vector2d& P_x_, const Vector2d& P_e_)
  {
    Vector2d P_x(P_x_);
    Vector2d P_e(P_e_);
    assert(P_x.x() >= 0);
    Vector2d T = P_e - P_x;
    T.normalize();
    if (P_e.x() > P_x.x())
      return acos(abs(T.x()));
    else      
      return M_PI - acos(abs(T.x()));
  }

  /**
  */
  double computeGamma(const Vec3d& P1, const Vec3d& P2, const Vec3d& P3)
  {
    return M_PI - wideAngle(P1-P2, P3-P2);
  }
  
  /** \brief approximation of constant c_4 on page 768, used in eq. (4)
  */
  const double C4 = 1.1228;
}

namespace gris
{
namespace muk
{
  /** \brief computes minLength

    see equation (4) in 
  */
  double BezierSpiralInterpolator::fromKappaAndGamma(double kappa, double gamma)
  {
    const auto beta = gamma / 2.0;
    const double cosBeta = cos(beta);
    const double sinBeta = sin(beta);
    return C4 * sinBeta/ (kappa * cosBeta * cosBeta);
  }

  /** \brief computes Gamma
    see equation (4) in 
  */
  double BezierSpiralInterpolator::fromKappaAndMinLength(double kappa, double minLength)
  {
    double q = minLength / C4 * kappa;
    double root = std::sqrt(1 / (q*q) + 4);
    double bigroot = std::sqrt(1 / (q*q) + 4/(q*root) + 1/ (q*q*q*root));
    return 4 * std::atan(0.5* root - bigroot / std::sqrt(2) + 0.5 / q);
  }

  /** \brief computes Kappa
    see equation (4) in 
  */
  double BezierSpiralInterpolator::fromMinLengthAndGamma(double minLength, double gamma)
  {
    const double cosBeta = cos(gamma / 2.0);
    return C4 * sin(gamma/2.0) / (minLength * cosBeta * cosBeta);
  }

  /**
  */
  void BezierSpiralInterpolator::approximateStepSize(double eps, double minGamma, double maxGamma, double stepSize, gris::muk::BezierSpiralInterpolator& spiralBuilder)
  {
    double length = spiralBuilder.getMinLength();
    double gamma  = spiralBuilder.getGamma();
    if ( abs(length-stepSize) > eps)
    {
      spiralBuilder.setGamma( 0.5*(minGamma + maxGamma) );
      if (length > stepSize)
      {
        maxGamma = gamma;
      }
      else
      {
        minGamma = gamma;
      }
      approximateStepSize(eps, minGamma, maxGamma, stepSize, spiralBuilder);
    }
  }

  /**
  */
  BezierSpiralInterpolator::BezierSpiralInterpolator()
    : mKappa(0.1)
    , mGamma(M_Pi_2)
  {
    mMinLength = fromKappaAndGamma(mKappa, mGamma);
  }

  /**
  */
  void BezierSpiralInterpolator::setGamma(double val)
  {
    mGamma = val;
  }

  /** \brief computes if input is valid for internal values of kappa, gamma and minLength

    see further: Yang et al: Spline-Based RRT Path Planner for Nonholonomic Robots
    see further: D.J. Walton: Planar G2 transition curves composed of cubic Bézier Spiral segments
    p.472 (Theorem 7)
  */
  bool BezierSpiralInterpolator::interpolatable(const Vec3d& W1, const Vec3d& W2, const Vec3d W3) const
  {    
    const double gamma = computeGamma(W1, W2, W3);
    if (gamma > mGamma)
    {
      return false;
    }
    const double length = std::min( (W1-W2).norm(), (W2-W3).norm());
    if (length < mMinLength)
    {
      return false;
    }
    return true;
  }

  /** \brief creates a spline connecting the points 1/2*(W1 + W2) and 1/2*(W2 + W3) via two Bezier Spirals
  */
  bool BezierSpiralInterpolator::interpolate(const Vec3d& W1, const Vec3d& W2, const Vec3d& W3, BezierSpline& spline) const
  {
    bool interpolationValid = false;
    using namespace Eigen;
    auto       P_1 = Vector3d(W1.data());
    const auto P_2 = Vector3d(W2.data());
    auto       P_3 = Vector3d(W3.data());

    // adjust points to smaller path    
    Vector3d u_t = (P_2-P_1); // unit tangent from W1 to W2, equation (17)    
    Vector3d u_p = (P_2-P_3); // unit tangent from W2 to W3, eq. (19)
    const double length_u_t = u_t.norm();
    const double length_u_p = u_p.norm();
    u_t.normalize();
    u_p.normalize();
    if (length_u_t < length_u_p) // change length P2 to P3
    {
      P_3 = P_2 - length_u_t*u_p;
    }
    else // change length P1 to P2
    {
      P_1 = P_2 - length_u_p*u_t;
    }

    // ---- construction of orthonormal Vectors ----    
    // binormal vector of P1P2P3-plane, eq. (18)    
    Vector3d u_b = u_t.cross(u_p);
    if (u_b.isZero()) // the three points form a line
    {
      spline.samples[0] = Vec3d(W1.x(), W1.y(), W1.z());
      spline.samples[3] = Vec3d(W2.x(), W2.y(), W2.z());
      spline.samples[6] = Vec3d(W3.x(), W3.y(), W3.z());
      const double db = (P_2-P_1).norm();
      spline.samples[1] = Convert()(P_1 + db / 3.0 * u_t);
      spline.samples[2] = Convert()(P_1 + 2 * db / 3.0 * u_t);
      const double de = (P_3-P_2).norm();
      spline.samples[4] = Convert()(P_2 - de / 3.0 * u_p);
      spline.samples[5] = Convert()(P_2 - 2 * de / 3.0 * u_p);
      interpolationValid = true;
      return true;
    }

    // unit normal vector, eq. (20)
    u_b.normalize();
    Vector3d u_n = u_b.cross(u_t);
    u_n.normalize();
    // ---- Mapping 3D Waypoints to 2D ----
    // transformation Matrix, eq. (21)
    Matrix4d TM = createTrafoMatrix(u_t, u_n, u_b, P_1);
    Matrix4d TMI = TM.inverse();

    // Applying 2D Bezier interpolation
    const Vector2d P_b = project_3d_2d(TMI, P_1);
    const Vector2d P_e = project_3d_2d(TMI, P_3);
    const Vector2d P_x = project_3d_2d(TMI, P_2);

    Vector2d T_b = P_x - P_b;
    Vector2d T_e = P_e - P_x;
    const double d = T_b.norm(); // always 0.5
    T_b.normalize();
    T_e.normalize();

    const double alpha = computeAlpha(P_x, P_e);
    const double phi = alpha / 2.0;
    const double sphi = sin(phi);
    const double cphi = cos(phi);
    const double d_max = ::C4 * sphi / (mKappa* cphi*cphi);
    if (d>=d_max)
    {
      interpolationValid = true;
    }

    const double h_b = 0.346 * d;  // eq. 16
    const double g_b = 0.58  * h_b;          // eq. 16
    const double k_b = 1.31  * h_b * cphi; // eq. 16

    // compute Points    
    const Vector2d& B0 = P_b;
    const Vector2d& E0 = P_e;
    Vector2d  B1, B2, B3, E1, E2;
    B1 = B0 + g_b * T_b;
    B2 = B1 + h_b * T_b;
    E1 = E0 - g_b * T_e;
    E2 = E1 - h_b * T_e;
    Vector2d u_d = E2 - B2;
    u_d.normalize();
    B3 = B2 + k_b * u_d;

    // Reprojection to 3D
    MatrixXd S2D = createTrafoMatrix(B0, B1, B2, B3, E2, E1, E0);
    MatrixXd S3D = TM * S2D;
        
    for (size_t i(0); i < 7; ++i)
    {
      spline.samples[i] = Vec3d(S3D(0, i), S3D(1, i), S3D(2, i));
      if (spline.samples[i].hasNaN())
      {
        interpolationValid = false;
      }
    }
    return interpolationValid;
  }
  
  ///** \brief creates a spline connecting the points 1/2*(W1 + W2) and 1/2*(W2 + W3) via two Bezier Spirals
  //  
  //  convenience function
  //*/
  //bool BezierSpiralInterpolator::interpolate(const Vec3d& W1, const Vec3d& W2, const Vec3d& W3, BezierSpline& spline) const
  //{
  //  using namespace Eigen;
  //  Vector3d P_1 = Vector3d(W1.data());
  //  Vector3d P_2 = Vector3d(W2.data());
  //  Vector3d P_3 = Vector3d(W3.data());
  //  return this->interpolate(P_1, P_2, P_3,  spline);
  //}

  /** \brief Creates a Bézier Spline that interpolates between B0 and E0.

    see further: Yang et al: Spline-Based RRT Path Planner for Nonholonomic Robots
  */
  BezierSpline BezierSpiralInterpolator::createFromEndpoints(const Vec3d& B0, const Vec3d& W2, const Vec3d E0)
  {
    BezierSpline spline;
    using namespace Eigen;
    // gris Vector class doesn't support matrix operations -> convert to eigen
    auto P_1        = Vector3d(B0.data());
    const auto P_2  = Vector3d(W2.data());
    auto P_3        = Vector3d(E0.data());
    // adjust points to smaller path
    Vector3d u_t = (P_2-P_1); // unit tangent from B0 to W2, equation (17)    
    Vector3d u_p = (P_2-P_3); // unit tangent from E0 to W2, eq. (19)
    const double length_u_t = u_t.norm();
    const double length_u_p = u_p.norm();
    u_t.normalize();
    u_p.normalize();
    if (length_u_t < length_u_p) // change length P2 to P3
      P_3 = P_2 - length_u_t*u_p;
    else // change length P1 to P2
      P_1 = P_2 - length_u_p*u_t;

    // ---- construction of orthonormal Vectors ----    
    // binormal vector of P1P2P3-plane, eq. (18)
    Vector3d u_b = u_t.cross(u_p);
    if (u_b.isZero()) // the three points form a line
    {
      spline.samples[0] = B0;
      spline.samples[3] = W2;
      spline.samples[6] = E0;
      const double db = (P_2-P_1).norm();
      spline.samples[1] = Convert()(P_1 + db / 3.0 * u_t);
      spline.samples[2] = Convert()(P_1 + 2 * db / 3.0 * u_t);
      const double de = (P_3-P_2).norm();
      spline.samples[4] = Convert()(P_2 - de / 3.0 * u_p);
      spline.samples[5] = Convert()(P_2 - 2 * de / 3.0 * u_p);
      return spline;
    }

    // unit normal vector, eq. (20)
    u_b.normalize();
    Vector3d u_n = u_b.cross(u_t);
    u_n.normalize();
    // ---- Mapping 3D Waypoints to 2D ----
    // transformation Matrix, eq. (21)
    Matrix4d TM = createTrafoMatrix(u_t, u_n, u_b, P_1);
    Matrix4d TMI = TM.inverse();

    // Applying 2D Bezier interpolation
    const Vector2d P_b = project_3d_2d(TMI, P_1);
    const Vector2d P_e = project_3d_2d(TMI, P_3);
    const Vector2d P_x = project_3d_2d(TMI, P_2);

    Vector2d T_b = P_x - P_b;
    Vector2d T_e = P_e - P_x;
    const double d = T_b.norm(); // always 0.5
    T_b.normalize();
    T_e.normalize();

    const double alpha = computeAlpha(P_x, P_e);
    const double phi = alpha / 2.0;
    const double sphi = sin(phi);
    const double cphi = cos(phi);
    const double h_b  = 0.346 * d;          // eq. 16
    const double g_b  = 0.58  * h_b;        // eq. 16
    const double k_b  = 1.31  * h_b * cphi; // eq. 16

    // compute Points    
    const Vector2d& B0_2D = P_b;
    const Vector2d& E0_2D = P_e;
    Vector2d  B1, B2, B3, E1, E2;
    B1 = B0_2D + g_b * T_b;
    B2 = B1 + h_b * T_b;
    E1 = E0_2D - g_b * T_e;
    E2 = E1 - h_b * T_e;
    Vector2d u_d = E2 - B2;
    u_d.normalize();
    B3 = B2 + k_b * u_d;

    // Reprojection to 3D
    MatrixXd S2D = createTrafoMatrix(B0_2D, B1, B2, B3, E2, E1, E0_2D);
    MatrixXd S3D = TM * S2D;

    for (size_t i(0); i < 7; ++i)
      spline.samples[i] = Vec3d(S3D(0, i), S3D(1, i), S3D(2, i));
    return spline;
  }

  /** \brief Creates a Bézier Spline that starts at 0.5*(W1+W2) and ends at 0.5*(W2+W3), point at start in (W2-W1) and at end in (W3-W2).
  */
  BezierSpline BezierSpiralInterpolator::createFromWaypoints(const Vec3d& W1, const Vec3d& W2, const Vec3d W3)
  {
    const auto B0 = 0.5*(W1 + W2);
    const auto E0 = 0.5*(W2 + W3);
    return createFromEndpoints(B0, W2, E0);
  }

  /**
  */
  double BezierSpiralInterpolator::computeKappaMax(const Vec3d& W1, const Vec3d& W2, const Vec3d& W3) const
  {
    const auto B0 = 0.5*(W1 + W2);
    const auto E0 = 0.5*(W2 + W3);
    const auto v1  = (W2 - W1).normalized();
    const auto v2  = (W3 - W2).normalized();
    const auto dot = v1.dot(v2);
    const auto angle = std::acos(std::max(-1.0,std::min(1.0,std::abs(dot))));
    const auto gamma = dot > 0 ? angle : M_Pi - angle;
    const auto beta  = gamma / 2.0;
    const auto dist1 = (W2 - B0).squaredNorm();
    const auto dist2 = (W2 - E0).squaredNorm();
    const auto d = dist1 < dist2 ? std::sqrt(dist1) : std::sqrt(dist2);
    const auto cbeta = std::cos(beta);
    const auto kappa = std::abs(gamma) < 1e-5 ? 0.0 : C4*std::sin(beta) / (d*cbeta*cbeta);
    return kappa;
  }
}
}