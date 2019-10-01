#include "private/muk.pch"
#include "geometry.h"
#include "muk_eigen_helper.h"
#include "MukException.h"

#include <Eigen/Dense>
#include "gris_math.h"

#include <boost/format.hpp>

#include <algorithm>
#include <numeric>
#include <cmath>

#ifdef M_PI
#undef M_PI
#endif

namespace
{ 
  using namespace gris;
  using namespace gris::muk;
  void approxInscribedCircle(const std::vector<Vec2d>& input, Vec2d& center, double& radius);
}

namespace gris
{
namespace muk
{
  /**
  */
  bool Circle2D::isInside(const Vec2d& p)
  {
    return (mCenter-p).squaredNorm() < mRadius*mRadius;
  }

  /**
  */
  Eigen::Matrix3d rotationMatrixOntoVector(const Vec3d& from_, const Vec3d& to_)
  {
    using namespace Eigen;
    const auto from = asEigen(from_.normalized());
    const auto to   = asEigen(to_.normalized());
    const auto v    = from.cross(to).normalized();
    const double s = v.norm();
    const double c = from.dot(to);
    Eigen::Matrix3d R;
    R(0, 0) = 0;   R(0, 1) = -v.z();   R(0, 2) = v.y();
    R(1, 0) = v.z();   R(1, 1) = 0;   R(1, 2) = -v.x();
    R(2, 0) = -v.y();   R(2, 1) = v.x();   R(2, 2) = 0;
    R = Eigen::Matrix3d::Identity() + R + (1 - c) / (s*s) * R*R;
    return R;
  }

  /** \brief returns the angle between two vectors, within [0, pi]

      angle in rad
  */
  double wideAngle(const Vec3d& v, const Vec3d& w)
  {
    auto T1 = v.normalized();
    auto T2 = w.normalized();
    return std::atan2(T1.cross(T2).norm(), T1.dot(T2));
  }
  /** \brief returns the angle between two vectors, projected onto 2D, within [0, 2*pi]

    angle in rad
  */
  double angleIn2D(const Vec3d& v, const Vec3d& w)
  {
    using namespace Eigen;
    auto v1 = asEigen(v.normalized());
    auto v2 = asEigen(w.normalized());      
    double angle = atan2(v2.y(), v2.x()) - atan2(v1.y(), v1.x());
    if (angle < 0)
      angle += 2 * M_Pi;
    return angle;
  }

  /**
  */
  Vec3d rotateOntoVector(const Vec3d& from_, const Vec3d& to_)
  {
    using namespace Eigen;
    Vector3d from(from_.data());
    from.normalize();
    Vector3d to(to_.data());
    to.normalize();
    Vector3d v = from.cross(to);
    v.normalize();
    const double c   = from.dot(to);
      
    Eigen::Matrix3d R;
    R(0,0) =      0;   R(0,1) = -v.z();   R(0,2) =  v.y();
    R(1,0) =  v.z();   R(1,1) =      0;   R(1,2) = -v.x();
    R(2,0) = -v.y();   R(2,1) =  v.x();   R(2,2) =      0;
    R = Eigen::Matrix3d::Identity() + R + (1-c) * R*R;
    Vector3d result = R*from;      
    Vec3d res(result.x(), result.y(), result.z());
    return res;
  }


  /**
    https://github.com/DLuensch/Least-Squares-Circle-Fitting-Kasa-Method-/blob/master/src/circleFitting.cpp
  */
  void fitCircle(const std::vector<Vec2d>& input, Circle2D& output)
  {
    int length = static_cast<int>( input.size() );
    double x1;
    double x2;
    double x3;
    Eigen::MatrixXd AFill(3, length);
    Eigen::MatrixXd A(length, 3);
    Eigen::VectorXd AFirst(length);
    Eigen::VectorXd ASec(length);
    Eigen::VectorXd AFirstSquared(length);
    Eigen::VectorXd ASecSquared(length);
    Eigen::VectorXd ASquaredRes(length);
    Eigen::VectorXd b(length);
    Eigen::VectorXd c(3);

    if (length > 1)
    {
      for (int i = 0; i < length; i++)
      {
        AFill(0, i) = input[i].x();
        AFill(1, i) = input[i].y();
        AFill(2, i) = 1;
      }

      A = AFill.transpose();

      for (int i = 0; i < length; i++)
      {
        AFirst(i) = A(i, 0);
        ASec(i) = A(i, 1);
      }

      for (int i = 0; i < length; i++)
      {
        AFirstSquared(i) = AFirst(i) * AFirst(i);
        ASecSquared(i) = ASec(i) * ASec(i);
      }

      b = AFirstSquared + ASecSquared;

      c = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

      Vec2d center;
      x1 = c(0);
      center.x() = x1 * 0.5;
      x2 = c(1);
      center.y() = x2 * 0.5;
      x3 = c(2);
      const double radius = sqrt((x1 * x1 + x2 * x2) / 4 + x3);
      output.setCenter(center);
      output.setRadius(radius);
    }
    else
    {
      output.setRadius(std::numeric_limits<double>::infinity());
    }
  }


  /** \brief fits a sphere into the points

    actually computes the circumcircle of the three points
  */
  void fitSphere(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, Sphere3D& output)
  {
    // http://en.wikipedia.org/wiki/Circumscribed_circle (see the part about "the circumcircle of a triangle embedded in d dimensions"):
    if (p1==p2 || p2==p3 || p1==p3)
    {
      /*std::stringstream ss1; ss1 << p1;
      std::stringstream ss2; ss2 << p2;
      std::stringstream ss3; ss3 << p3;
      const auto str = (boost::format("p1=%s, p2=%s p3=%s)") % ss1.str() % ss2.str() % ss3.str()).str();
      LOG_LINE << "Warning: Some points are equal." << str;*/
      output.setCenter(p2);
      output.setRadius(std::min( (p1-p2).norm(), (p3-p2).norm()) ); // TODO fix this whole thing
    }
    else
    {
      using namespace Eigen;
      const Vec3d tmpa = p1-p3;
      const Vec3d tmpb = p2-p3;
      const Vector3d a(tmpa.data());
      const Vector3d b(tmpb.data());
      const double squaredNormA = a.squaredNorm();
      const double squaredNormB = b.squaredNorm();
      const double normA = std::sqrt(squaredNormA);
      const double normB = std::sqrt(squaredNormB);
      const Vector3d abcross = a.cross(b);
      const double radius = normA*normB* (a-b).norm() / (2 * abcross.norm());
      const auto center =  (1.0 / (2*abcross.squaredNorm())) * (squaredNormA*b - squaredNormB*a).cross(abcross) + Vector3d(p3.data());
      if (radius < 10e-5)
      {
        output.setRadius(std::numeric_limits<double>::infinity());
      }
      else
      {
        output.setRadius(radius);
      }
      output.setCenter(Vec3d(center.x(), center.y(), center.z()));
    }
  }


  void setCol(Eigen::Matrix4d& m, size_t i, const Vec3d& p, double d)
  {
    m(0, i) = p.x();
    m(1, i) = p.y();
    m(2, i) = p.z();
    m(3, i) = d;
  }

  void setRow(Eigen::Matrix4d& m, size_t i, const Vec3d& p, double d)
  {
    m(i, 0) = p.x();
    m(i, 1) = p.y();
    m(i, 2) = p.z();
    m(i, 3) = d;
  }

  /** \brief

    see further http://paulbourke.net/geometry/circlesphere/spheretest.c
  */
  void fitSphere(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, const Vec3d& p4, Sphere3D& output)
  {
    Eigen::Matrix4d M;
    setRow(M, 0, p1, 1);
    setRow(M, 1, p2, 1);
    setRow(M, 2, p3, 1);
    setRow(M, 3, p4, 1);      
    double detM11 = M.determinant();
    if (detM11 == 0)
    {
      LOG_LINE << "The points don't define a sphere!";
    }
      
    M(0,0) = p1.squaredNorm();
    M(1,0) = p2.squaredNorm();
    M(2,0) = p3.squaredNorm();
    M(3,0) = p4.squaredNorm();
    double detM12 = M.determinant();

    M(0,1) = p1.x();
    M(1,1) = p2.x();
    M(2,1) = p3.x();
    M(3,1) = p4.x();
    double detM13 = M.determinant();

    M(0,2) = p1.y();
    M(1,2) = p2.y();
    M(2,2) = p3.y();
    M(3,2) = p4.y();
    double detM14 = M.determinant();

    M(0,3) = p1.z();
    M(1,3) = p2.z();
    M(2,3) = p3.z();
    M(3,3) = p4.z();
    double detM15 = M.determinant();
      
    LOG_LINE << "Determinants: " << detM11 << " " << detM12 << " " << detM13 << " " << detM14 << " " << detM15 << " ";

    const auto center = Vec3d(0.5 * detM12 / detM11, 0.5 * detM13 / detM11, 0.5 * detM14 / detM11);
    const double r = sqrt(center.dot(center) - detM15/detM11);
  }
    
  /** \brief fits a plane through the points
  */
  void fitPlane(const VecN3d& points, Plane3D& plane)
  {
    using namespace Eigen;
    // solve least squares problem
    std::vector<double> x, y, z;
    std::transform(points.begin(), points.end(), back_inserter(x), [&] (const auto& v) { return v.x(); });
    std::transform(points.begin(), points.end(), back_inserter(y), [&] (const auto& v) { return v.y(); });
    std::transform(points.begin(), points.end(), back_inserter(z), [&] (const auto& v) { return v.z(); });
    double xm = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
    double ym = std::accumulate(y.begin(), y.end(), 0.0) / x.size();
    double zm = std::accumulate(z.begin(), z.end(), 0.0) / x.size();
    std::transform(x.begin(), x.end(), x.begin(), [&] (const double d) { return d-xm; });
    std::transform(y.begin(), y.end(), y.begin(), [&] (const double d) { return d-ym; });
    std::transform(z.begin(), z.end(), z.begin(), [&] (const double d) { return d-zm; });
    typedef Eigen::Matrix3d Matrix;
    Matrix M;
    std::vector<double> sums(x.size());
    {
      std::transform(x.begin(), x.end(), sums.begin(), [] (double d) { return d*d; });
      M(0, 0) = std::accumulate(sums.begin(), sums.end(), 0.0);
      std::transform(x.begin(), x.end(), y.begin(), sums.begin(), [] (double d1, double d2) { return d1*d2; });
      M(0, 1) = std::accumulate(sums.begin(), sums.end(), 0.0);
      std::transform(x.begin(), x.end(), z.begin(), sums.begin(), [] (double d1, double d2) { return d1*d2; });
      M(0, 2) = std::accumulate(sums.begin(), sums.end(), 0.0);
    }
    {
      std::transform(y.begin(), y.end(), sums.begin(), [] (double d) { return d*d; });
      M(1, 1) = std::accumulate(sums.begin(), sums.end(), 0.0);
      std::transform(y.begin(), y.end(), x.begin(), sums.begin(), [] (double d1, double d2) { return d1*d2; });
      M(1, 0) = std::accumulate(sums.begin(), sums.end(), 0.0);
      std::transform(y.begin(), y.end(), z.begin(), sums.begin(), [] (double d1, double d2) { return d1*d2; });
      M(1, 2) = std::accumulate(sums.begin(), sums.end(), 0.0);
    }
    {
      std::transform(z.begin(), z.end(), sums.begin(), [] (double d) { return d*d; });
      M(2, 2) = std::accumulate(sums.begin(), sums.end(), 0.0);
      std::transform(z.begin(), z.end(), x.begin(), sums.begin(), [] (double d1, double d2) { return d1*d2; });
      M(2, 0) = std::accumulate(sums.begin(), sums.end(), 0.0);
      std::transform(z.begin(), z.end(), y.begin(), sums.begin(), [] (double d1, double d2) { return d1*d2; });
      M(2, 1) = std::accumulate(sums.begin(), sums.end(), 0.0);
    }
    EigenSolver<Matrix> solver(M);
    auto lambdas = solver.eigenvalues();
    auto v = solver.eigenvectors();
    plane.setOrigin(Vec3d(xm, ym, zm));
    plane.setNormal(Vec3d(v(0, 0).real(), v(1, 0).real(), v(2, 0).real()));
  }

  /** \brief calculates samples on a radius r around center, circle/ disk perpendicular to tangent

    \param center center of circle
    \param tangent normal, perpendicular to the "disk" of the circle
    \param r Radius of the circle
    \param samples output / result with the sample points
  */
  void circleAroundLine(const Vec3d& center, const Vec3d& tangent, double r, std::vector<Vec3d>& samples)
  {
    if (center.hasNaN())
    {
      std::stringstream ss;
      ss << center;
      throw MUK_EXCEPTION("invalid input vector 'center'", ss.str().c_str());
    }
    if (tangent.hasNaN() || tangent.isZero())
    {
      std::stringstream ss;
      ss << tangent;
      throw MUK_EXCEPTION("invalid input vector 'tangent'", ss.str().c_str());
    }
    if (samples.empty())
      return;

    auto circle = Circle3D();
    circle.setRadius(r);
    circle.setCenter(center);
    circle.setNormal(tangent);

    const size_t N = samples.size();
    const double stepSize = 2.0 * M_Pi / N;
    for (size_t i(0); i<N; ++i)
    {
      const double t = i*stepSize;        
      samples[i] = circle.interpolate(t);
    }
  }

  /**
  */
  void Circle3D::setXAxis(const Vec3d& v)
  {
    mxAxis = v;
    myAxis = mNormal.cross(v).normalized();
    mInitialized = true;
  }

  /**
  */
  void Circle3D::initialize() const
  {
    double d[3] = { abs(mNormal.x()), abs(mNormal.y()), abs(mNormal.z()) };
    int dimension = static_cast<int>( std::distance(d, std::min_element(d, d+3)) );
    Vec3d helperVector;
    switch (dimension)
    {
      case 0:
        helperVector = Vec3d(1, 0, 0); break;
      case 1:
        helperVector = Vec3d(0, 1, 0); break;
      case 2:
        helperVector = Vec3d(0, 0, 1); break;
    }
            
    mxAxis = mNormal.cross(helperVector);
    mxAxis.normalize();
    myAxis = mxAxis.cross(mNormal);
    myAxis.normalize();
    mInitialized = true;
  }
     
  /**
  */
  Vec3d Circle3D::interpolate(double t) const
  {
    if ( ! mInitialized)
    {
      initialize();
    }
    // phi(t) := x0 + r*cos(t)*p + r*sin(t)*q.
    return mCenter + mRadius*cos(t)*mxAxis + mRadius*sin(t)*myAxis;
  }

  /**
  */
  std::vector<Vec3d> Circle3D::interpolate(double startAngle, double goalAngle, double resolution) const
  {
    if ( ! mInitialized)
    {
      initialize();
    }
    // phi(t) := x0 + r*cos(t)*p + r*sin(t)*q.
    std::vector<Vec3d> result;
    for (double t(startAngle); t<=goalAngle; t+=resolution)
      result.push_back(mCenter + mRadius*cos(t)*mxAxis + mRadius*sin(t)*myAxis);
    return result;
  }

  /**
  */
  std::vector<Vec3d> Circle3D::interpolate(double startAngle, double goalAngle, size_t numPoints) const
  {
    if ( ! mInitialized)
    {
      initialize();
    }
    // phi(t) := x0 + r*cos(t)*p + r*sin(t)*q.
    std::vector<Vec3d> result;
    const auto dt = (goalAngle-startAngle) / numPoints;
    for (double t(startAngle); t<=goalAngle; t+=dt)
      result.push_back(mCenter + mRadius*cos(t)*mxAxis + mRadius*sin(t)*myAxis);
    return result;
  }

  /**
  */
  Vec3d Circle3D::direction(double t) const
  {
    if ( ! mInitialized)
    {
      initialize();
    }
    // dphi/dt = -r*sin(t)*p + r*cos(t)*q.
    auto v = -mRadius*sin(t)*mxAxis + mRadius*cos(t)*myAxis;
    v.normalize();
    return v;
  }

  /**
  */
  double Circle3D::angle(const Vec3d& v) const
  {
    if ( ! mInitialized)
    {
      initialize();
    }
    auto dotx = mxAxis.dot(v);
    auto doty = myAxis.dot(v);
    double angle = std::acos(abs(dotx));
    if (dotx>=0)
    {
      if (doty>=0)
      {
        return angle;
      }
      else
      {
        return 2*M_Pi - angle;
      }
    }
    else
    {
      if (doty>=0)
      {
        return M_Pi-angle;
      }
      else
      {
        return M_Pi+angle;
      }
    }
  }
    
  /**
  */
  Line3D::Line3D()
    : mOrigin(0,0,0)
    , mDirection(0,0,0)
  {
  }

  /**
  */
  Line3D::Line3D(const Vec3d& origin, const Vec3d& direction)
    : mOrigin(origin)
    , mDirection(direction)
  {
  }

  /**
  */
  Line3D::~Line3D()
  {
  }

  /**
  */
  Line3D::Query Line3D::intersects(const Line3D& line) const
  {
    Query result;
    intersects(line, result);
    return result;
  }
      
  /**
  */
  void Line3D::intersects(const Line3D& line, Query& query) const
  {
    using namespace Eigen;
    Vector3d P(asEigen(mOrigin));
    Vector3d Tp(asEigen(mDirection));

    Vector3d Q(asEigen(line.getOrigin()));
    Vector3d Tq(asEigen(line.getDirection()));

    Vector3d binormal = Tp.cross(Tq);
    bool isColinear = binormal.isZero();
    if (isColinear)
    {
      query.intersectionType = Query::enParallel;
      query.intersectionPoint1 = asMuk(0.5*(P+Q));
      query.intersectionPoint2 = asMuk(0.5*(P+Q));
      return;
    }
    binormal.normalize();

    // normal of plane, that line_e will cross
    const Vector3d Nq = binormal.cross(Tq).normalized();
    ParametrizedLine<double, 3> line_p(P, Tp);
    Hyperplane      <double, 3> hp_q(Nq, Q);
    query.intersectionPoint1 = asMuk(line_p.intersectionPoint(hp_q));

    // normal of plane, that line_b will cross
    const Vector3d Np = binormal.cross(Tp).normalized();
    ParametrizedLine<double, 3> line_q(Q, Tq);
    Hyperplane      <double, 3> hp_p(Np, P);
    query.intersectionPoint2 = asMuk(line_q.intersectionPoint(hp_p));
    query.intersectionType = Query::enSkewLines;
  }

  /** \brief returns the projection of point p onto the line.

    general formula: A + dot(AP,AB) / dot(AB,AB) * AB
    use this with 
      A = mOrigin
      B = mOrigin + mDirection
      AB = mDirection;

    \param p The point that should be projected onto the line.
    \return The projected point.
  */
  Vec3d Line3D::project(const Vec3d& p) const
  {
    //const auto B  = mOrigin + mDirection;
    const auto AP = p - mOrigin;
    return mOrigin + AP.dot(mDirection) * mDirection; // dot(AB,AB) = 1
  }

  /** \brief returns true if the query point p lies within the cone, false otherwise.
  */
  bool Cone3D::isInside(const Vec3d& p) const
  {
    // project onto center line.
    const double dot = (p-mConeTip).dot(mConeDirection);
    if (dot < 0)
      return false;
    const auto   q    = mConeTip + dot*mConeDirection;
    const auto   dist = (q - mConeTip).norm();
    // far away from tip?
    if (dist > mLength)
      return false;
    const double r = (dist / mLength) * mRadius;
    const double d = (p - q).norm();        
    // outside cone ?
    if (d > r)
      return false;
    return true;
  }
}
}

#pragma warning(push)
#pragma warning( disable : 4996 ) // sscanf
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/extremal_polygon_2.h>
#pragma warning(pop)

namespace gris
{
  namespace muk
  {
    
  /** \brief calculates the points of <input> that define the inscribed polygon

    requires that the points lie on a plane   
    see further http://doc.cgal.org/latest/Inscribed_areas/index.html#Chapter_Inscribed_Areas

    \return output: the points of the inscribed polygon, sorted
  */
  MUK_COMMON_API void inscribedPolygon(const std::vector<Vec3d>& input, std::vector<Vec3d>& output)
  {
    //does not work

    //// fit a plane
    //Plane3D plane;
    //fitPlane(input, plane);
    //// transform onto xy plane
    //using namespace Eigen;
    //Eigen::Transform<double,3,Affine> Trafo;
    //std::vector<Vector3d> work;
    //{
    //  Vector3d nUp(1,0,0);
    //  Vector3d nPlane(plane.getNormal().x(), plane.getNormal().y(), plane.getNormal().z);
    //  Vector3d axis = nUp.cross(nPlane);
    //           axis.normalize();
    //  const double angle = acos( nUp.dot(nPlane) );
    //  AngleAxisd R(angle, axis);
    //  Translation3d T(plane.getOrigin().x(), plane.getOrigin().y(), plane.getOrigin().z);
    //  Trafo = R*T;      
    //  std::transform(input.begin(), input.end(), std::back_inserter(work), [&] (const Vec3d& p) { return Vector3d(p.x(), p.y(), p.z); });
    //  std::for_each(work.begin(), work.end(), [&] (Vector3d& p) { p = Trafo*p; });
    //  LOG_LINE << "point set size " << work.size();
    //}
    //// compute inscribed polygon
    //{
    //  // CGAL kernel definitios
    //  typedef double                     FT;
    //  typedef CGAL::Simple_cartesian<FT> Kernel;
    //  typedef Kernel::Point_2            Point;
    //  typedef std::vector<int>           Index_cont;
    //  typedef CGAL::Polygon_2<Kernel>    Polygon_2;
    //  // compute maximum area incribed k-gon      
    //  std::vector<Point> data;
    //  std::transform(work.begin(), work.end(), std::back_inserter(data), [&] (const Vector3d& p) { return Point(p.x(), p.y()); });
    //  Polygon_2 k_gon (data.begin(), data.end());
    //  LOG_LINE << "k_gon size " << k_gon.size();
    //}
    //// backproject
    //Trafo = Trafo.inverse();
    //std::for_each(work.begin(), work.end(), [&] (Vector3d& p) { p = Trafo*p; });
  }

  /**
  */
  void approxInscribedCircle(const std::vector<Vec3d>& input, Vec3d& output)
  {
    // fit a plane
    Plane3D plane;
    fitPlane(input, plane);
    // transform onto xy plane
    using namespace Eigen;
    Eigen::Transform<double,3,Affine> Trafo;
    std::vector<Vector3d> work;
    Vector3d futureCenter;
    {
      Vector3d nUp(0,0,1);
      Vector3d nPlane(plane.getNormal().x(), plane.getNormal().y(), plane.getNormal().z());
      Vector3d axis = nUp.cross(nPlane);
               axis.normalize();
      double angle = acos( nUp.dot(nPlane) );

      auto R = rotationMatrixOntoVector(plane.getNormal(), Vec3d(0,0,1));
      //rotateOntoVector(plane.getNormal(), Vec3d(0,0,1));

      //AngleAxisd R(angle, axis);
      Translation3d T(-plane.getOrigin().x(), -plane.getOrigin().y(), -plane.getOrigin().z());
      Trafo = R*T;      
      std::transform(input.begin(), input.end(), std::back_inserter(work), [&] (const Vec3d& p) { return Vector3d(p.data()); });
      std::for_each(work.begin(), work.end(), [&] (Vector3d& p) { p = Trafo*p; });
      /*LOG << "p:\n  ";
      std::for_each(work.begin(), work.end(), [&] (Vector3d& p) { LOG << p.z() << " "; } );
      LOG_LINE << "";*/
    }
    {
      std::vector<Vec2d> tmp;
      std::transform(work.begin(), work.end(), std::back_inserter(tmp), [&] (const Vector3d& p) { return Vec2d(p.x(), p.y()); });
      Vec2d p;
      double r;
      ::approxInscribedCircle(tmp, p, r);
      futureCenter = Vector3d(p.x(), p.y(), 0);
    }
    // backproject
    Trafo = Trafo.inverse();
    //std::for_each(work.begin(), work.end(), [&] (Vector3d& p) { p = Trafo*p; });
    futureCenter = Trafo * futureCenter;
    output.x() = futureCenter.x();
    output.y() = futureCenter.y();
    output.z() = futureCenter.z();    
  }

  /** \brief computes the maximum curvature constraint

    assume the robot to have overall length "length", has a revolute joint in the middle and a maximum turning angle of "maxAngle"
    return the inverse of the radius of a circle through three points of ...
    \param maxAngle maximum angle constraint, in radians
    \param length   overall length of the robot, in millimeters
    \return output  computed curvature constraint
  */
  void maxCurvatureFromMaxAngle(double maxAngle, double length, double& output)
  {
    Vec3d origin(0,0,0);    
    const double l = 0.5*length;
    const double sphi = sin(maxAngle);
    const double cphi = cos(maxAngle);
    const Vec3d rhs = l * Vec3d(cphi, sphi, 0);
    const Vec3d lhs = l * Vec3d(-cphi, sphi, 0);
    Sphere3D sphere;    
    fitSphere(lhs, origin, rhs, sphere);
    output = 1.0 / sphere.getRadius();
  }
}
}

namespace
{
  /**
  */
  double dist(const std::vector<Vec2d>& input, Vec2d& query)
  {
    if (input.empty())
      return 0;
    auto iter = min_element(input.begin(), input.end(), [&] (const Vec2d& p, const Vec2d& q) { return (p-query).squaredNorm() < (q-query).squaredNorm(); });
    return (query-*iter).norm();
  }
    
  /** quick solution
  */
  Vec2d minArea(const std::vector<Vec2d>& input, const Vec2d& p, double extent)
  {
    if (extent<0)
      throw MUK_EXCEPTION_SIMPLE("input < 0");
    if (extent<0.1)
      return p;    
    Vec2d start = p - 0.5*Vec2d(extent,extent);
    const size_t N = 20;
    const double stepSize = extent/(1.0*N);
    size_t imax(0);
    size_t jmax(0);
    double maxDist = 0;//std::numeric_limits<double>::max();
    for (size_t i(0); i<N; ++i)
    {
      for (size_t j(0); j<N; ++j)
      {
        Vec2d p = start + Vec2d (stepSize*i, stepSize*j);
        const double d = dist(input, p);
        if (d>maxDist)
        {
          maxDist = d;
          imax = i;
          jmax = j;
        }
      }
    }
    Vec2d currentMax = start + Vec2d (stepSize*imax, stepSize*jmax);
    return minArea(input, currentMax, stepSize);
  }


  /**

  see further
  http://stackoverflow.com/questions/4279478/largest-circle-inside-a-non-convex-polygon
  */
  void approxInscribedCircle(const std::vector<Vec2d>& input, Vec2d& center, double& radius)
  {
    Circle2D circle;
    fitCircle(input, circle);
    if (circle.getRadius() == std::numeric_limits<double>::infinity())
      throw MUK_EXCEPTION("error in fittin of circle", "input size was probably zero");

    center = minArea(input, center, circle.getRadius());
    radius = dist(input, center);
  }
}

// CGAL definitions for the plane fitting that didnt work
//
//#pragma warning( push )
//#pragma warning( disable : 4996 )
//// used internally by cgal
//#ifndef CGAL_EIGEN3_ENABLED
//#define CGAL_EIGEN3_ENABLED
//#endif
//
//#include <CGAL/Simple_cartesian.h>
////#include <CGAL/Epick_d.h>
////#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/linear_least_squares_fitting_3.h>
//
//#pragma warning( pop )
//
//namespace
//{
//  typedef double FT;
//  typedef CGAL::Simple_cartesian<double> K;
//  typedef K::Point_3 Point;
//  typedef K::Plane_3 Plane;
//  typedef K::Line_3  Line;
//  typedef K::Triangle_3 Triangle;
//}
//
//
