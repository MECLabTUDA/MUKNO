#include "private/muk.pch"
#include "BezierToArcTrajectory.h"

#include "MukCommon/MukException.h"
#include "MukCommon/Quaternion.h"

#include <Eigen/Geometry>

#include <random>

namespace
{
  using namespace gris;
  using namespace gris::muk;
        
  /** \brief transform one coordinate system into another

    code see https://stackoverflow.com/questions/15252919/how-to-find-the-transformation-matrix-of-a-change-of-basis-with-eigen
  */
  void findTransformBetween2CS(se3::Pose3D& pose, const Vec3d& fr0, const Vec3d& fr1, const Vec3d& fr2, const Vec3d& to0, const Vec3d& to1, const Vec3d& to2) 
  {
    typedef Eigen::Affine3d Transformation;
    using namespace Eigen;

    Transformation T, T2, T3 = Transformation::Identity();
    // Axes of the coordinate system "fr"
    const auto x1 = (fr1 - fr0).normalized(); // the versor (unitary vector) of the (fr1-fr0) axis vector
    const auto y1 = (fr2 - fr0).normalized();
    // Axes of the coordinate system "to"
    const auto x2 = (to1 - to0).normalized();
    const auto y2 = (to2 - to0).normalized();
    // transform from CS1 to CS2 
    // Note: if fr0==(0,0,0) --> CS1==CS2 --> T2=Identity
    T2.linear() << Vector3d(x1.data()), Vector3d(y1.data()), Vector3d(x1.cross(y1).data()); 
    // transform from CS1 to CS3
    T3.linear() << Vector3d(x2.data()), Vector3d(y2.data()), Vector3d(x2.cross(y2).data()); 
    // T = transform to CS2 to CS3
    // Note: if CS1==CS2 --> T = T3
    T.linear() = T3.linear() * T2.linear().inverse(); 
    auto& trafo =  T.linear();
    for (int i(0); i < 3; ++i)
      for (int j(0); j < 3; ++j)
        pose(i, j) = trafo(i, j);
  }

  /**
  */
  void setOrientation(se3::Pose3D& pose, const Vec3d& origin, const Vec3d& zAxis, const Vec3d& xAxis)
  {
    const auto yAxis = zAxis.cross(xAxis).normalized();

    const auto fr0 = Vec3d(0,0,0);
    const auto fr1 = Vec3d(1,0,0);
    const auto fr2 = Vec3d(0,1,0);

    const auto to0 = origin;
    const auto to1 = origin + xAxis;
    const auto to2 = origin + yAxis;

    findTransformBetween2CS(pose, fr0, fr1, fr2, to0, to1, to2);
  }

  /**
  */
  void setOrientation(se3::Pose3D& pose, const Vec3d& zAxis)
  {
    const auto orient = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(zAxis.data()), Eigen::Vector3d::UnitZ()).inverse();
    pose.setOrientation(Quaternion(orient.w(),orient.x(),orient.y(),orient.z()));
  }
}

namespace gris
{
namespace muk
{
  /**
  */
  BezierToArcTrajectory::Segment::~Segment()
  {
  }

  /**
  */
  BezierToArcTrajectory::LineSegment::~LineSegment()
  {
  }

  /**
  */
  BezierToArcTrajectory::CircleSegment::~CircleSegment()
  {
  }

  /**
  */
  se3::Pose3D BezierToArcTrajectory::LineSegment::interpolate(double t) const  
  {
    se3::TwistVector w;
    w[5] = mPhi;
    if (mPhi != 0)
      return (mPose * mTwist.hat().push(t))  *w.hat().push(t);
    else
      return mPose * mTwist.hat().push(t);
  }

  /**
  */
  void BezierToArcTrajectory::LineSegment::computePush()
  {
    mTwist[2] = length();
  }

  /**
  */
  se3::Pose3D BezierToArcTrajectory::CircleSegment::interpolate(double t) const  
  {
    se3::TwistVector w;
    w[5] = mPhi;
    return (mPose * mTwist.hat().push(t)) * w.hat().push(t);
  }

  /**
  */
  void BezierToArcTrajectory::CircleSegment::computePush()
  {
    mTwist[2] = length();
    mTwist[3] = length() / mCircle.getRadius();
  }

  /**
  */
  void BezierToArcTrajectory::Segment::computeRotation(const Pose3D& poseTarget)
  {
    auto poseFrom = interpolate(1.0);    
    auto x = Vec4d();
    x.y() = 1.0;
    const auto v1  = poseFrom.data()*x;
    const auto v2  = poseTarget.data()*x;
    const auto phi = wideAngle(Vec3d(v1.data()), Vec3d(v2.data()));
    //LOG_LINE << "phi " << phi;
    mPhi = phi;
  }


  // ---------------------------------------------------------------------------------

  /**
  */
  BezierToArcTrajectory::~BezierToArcTrajectory()
  {
  }


  /** \brief computes the individual segments along the waypoints
  */
  void BezierToArcTrajectory::compute()
  {
    mSegments.clear();
    std::vector<Vec3d>    newPoints;
    // resolve easy special case
    if (isEasyCase())
      return;
    // -> now we have at least one spline
    // prepare basic parameters
    const auto rMin = 1.0 / mKappa;
    {
      // first segment is always a line
      auto pObj  = std::make_unique<LineSegment>(mWaypoints[0], 0.5*(mWaypoints[0] + mWaypoints[1]));
      auto& line = *pObj;
      line.mPose.setPosition(line.mOrigin);
      setOrientation(line.mPose, line.mDirection);
      mSegments.push_back(std::move(pObj));
    }
    // start iterating through the spline waypoints
    const auto& types  = mHints;
    const auto N_Types = types.size();
    auto iter          = types.begin();
    if (types.empty())
    {
      throw MUK_EXCEPTION_SIMPLE("Incompatible MukPath (no hints available)");
    }
    for (size_t i(1); i<mWaypoints.size()-1; ++i, ++iter)
    {
      // basic variables
      const auto& w1 = mWaypoints[i-1];
      const auto& w2 = mWaypoints[i];
      const auto& w3 = mWaypoints[i+1];
      const auto B0  = 0.5*(w1 + w2);
      const auto E0  = 0.5*(w2 + w3);
      auto       tB  = (w2 - B0);
      const auto dB  = tB.norm();
      tB /= dB;
      auto       tE = (E0 - w2);
      const auto dE = tE.norm();
      tE /= dE;
      const auto normal = tB.cross(tE).normalized();
      // compute the pose
      const auto pose = getPose(B0, tB, normal);
      // compute circular arc transition
      if (normal.hasNaN())
      {
        // just a linear trajectory
        auto pObj = std::make_unique<LineSegment>(B0, E0);
        pObj->mPose = pose;
        mSegments.push_back(std::move(pObj));
      }
      else
      {
        double rMax(0.0); // initialize here
        const bool  startIsShort    = dB < dE;
        const bool  hasSameDistance = std::abs(dB - dE) < 10e-5;
        const auto  dmax  = startIsShort ? dB : dE;
        const auto& Pmax  = startIsShort ? B0 : E0;
        const auto  B0max = w2 - dmax*tB;
        const auto  E0max = w2 + dmax*tE;
        // some kind of circular arc
        const auto lineB  = Line3D(B0max, tB.cross(normal).normalized());
        const auto lineE  = Line3D(E0max, tE.cross(normal).normalized());
        const auto query  = lineB.intersects(lineE);
        if (10e-5 < (query.intersectionPoint2 - query.intersectionPoint1).squaredNorm())
        {
          // this can't really happen ...
          throw MUK_EXCEPTION_SIMPLE("inexact Spline to Circular Arc computation (center_max)");
        }
        // center of largest circular arc.
        const bool isParallel = query.intersectionType == Line3D::Query::enParallel;
        if (isParallel)
        {
          // just a linear trajectory
          auto pObj = std::make_unique<LineSegment>(B0, E0);
          pObj->mPose = pose;
          mSegments.push_back(std::move(pObj));
        }
        else
        {
          const auto C_max = query.intersectionPoint1;
          rMax       = (Pmax-C_max).norm();
          if (*iter == 0)
          {
            // a line segment has to be added at the start or the beginning
            if ( (! startIsShort) && (! hasSameDistance))
            {
              auto pObj = std::make_unique<LineSegment>(B0, B0max);
              pObj->mPose = pose;
              mSegments.push_back(std::move(pObj));
            }
            // arc max
            Circle3D circle;
            circle.setCenter(C_max);
            circle.setRadius(rMax);
            circle.setNormal(normal);
            const auto CB = (B0max - C_max).normalized();
            const auto CE = (E0max - C_max).normalized();
            circle.setXAxis(CB);
            auto pObj = std::make_unique<CircleSegment>();
            pObj->mCircle   = circle;
            pObj->mMaxAngle = wideAngle(CB,CE);
            pObj->mPose     = pose;
            mSegments.push_back(std::move(pObj));
            if ( startIsShort && ! hasSameDistance)
            {
              auto pObj = std::make_unique<LineSegment>(E0max, E0);
              pObj->mPose = pose;
              mSegments.push_back(std::move(pObj));
            }
          }
          else
          {
            // -> try arc min
            // incept theorem vmax/vmin = dE / dx
            const double dmin = dmax * rMin / rMax;
            // create minimal circle
            const auto B0min = w2 - dmin*tB;
            const auto E0min = w2 + dmin*tE;
            if ( (! startIsShort) && (! hasSameDistance) )
            {
              auto pObj = std::make_unique<LineSegment>(B0, B0min);
              pObj->mPose = pose;
              mSegments.push_back(std::move(pObj));
            }
            const auto lineB  = Line3D(B0min, tB.cross(normal));
            const auto lineE  = Line3D(E0min, tE.cross(normal));
            const auto query  = lineB.intersects(lineE);
            if (10e-5 < (query.intersectionPoint2 - query.intersectionPoint1).squaredNorm())
            {
              throw MUK_EXCEPTION_SIMPLE("inexact Spline to Circular Arc computation (center_min)");
            }
            // center of smallest circular arc.
            const auto isParallel = query.intersectionType == Line3D::Query::enParallel;
            if (isParallel)
            {
              // just a linear trajectory
              auto pObj = std::make_unique<LineSegment>(B0, E0);
              pObj->mPose = pose;
              mSegments.push_back(std::move(pObj));
            }
            else
            {
              const auto C_min = query.intersectionPoint1;
              Circle3D circle;
              circle.setCenter(C_min);
              circle.setRadius(rMin);
              circle.setNormal(normal);
              const auto CB = (B0min - C_min).normalized();
              const auto CE = (E0min - C_min).normalized();
              circle.setXAxis(CB);
              auto pObj = std::make_unique<CircleSegment>();
              pObj->mCircle   = circle;
              pObj->mMaxAngle = wideAngle(CB,CE);
              pObj->mPose     = pose;
              pObj->mTwist[2] = pObj->length();
              pObj->mTwist[3] = pObj->length() / circle.getRadius();
              auto phi (0.0);
              {
              }
              pObj->mTwist[5] = phi;
              mSegments.push_back(std::move(pObj));
              if ( startIsShort && ! hasSameDistance)
              {
                auto pObj   = std::make_unique<LineSegment>(E0min, E0);
                pObj->mPose = pose;
                mSegments.push_back(std::move(pObj));
              }
            }
          }
        }
      }
    }
    {
      // last segment is always a line
      const auto& last   = mWaypoints.back();
      const auto  lastE0 = 0.5*(mWaypoints[mWaypoints.size() - 2] + last);
      auto pObj = std::make_unique<LineSegment>(lastE0, last);
      pObj->mPose.setPosition(lastE0);
      setOrientation(pObj->mPose, pObj->mDirection);
      mSegments.push_back(std::move(pObj));
    }
    for (auto& seg : mSegments)
      seg->computePush();
    for (size_t i(1); i<mSegments.size(); ++i)
      mSegments[i-1]->computeRotation(mSegments[i]->mPose);
  }

  /** \brief computes the pose at the current position B0

    \return a pose in SE(3)
  */
  se3::Pose3D BezierToArcTrajectory::getPose(const Vec3d& B0, const Vec3d& zAxis, const Vec3d& yAxis)
  {
    se3::Pose3D pose;
    Vec3d y;
    if (yAxis.hasNaN())
    {
      double lower_bound = 0;
      double upper_bound = 1;
      std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
      std::default_random_engine re;
      y = Vec3d(unif(re), unif(re), unif(re)).normalized();
    }
    else
    {
      y = yAxis;
    }
    //const auto x   = zAxis.cross(y).normalized();
    pose.setPosition(B0);
    setOrientation(pose, B0, zAxis, y);
    return pose;
  }

  /**
  */
  bool BezierToArcTrajectory::isEasyCase()
  {
    if (mWaypoints.size() < 2)
      return true;
    if (mWaypoints.size() == 2)
    {
      auto pObj  = std::make_unique<LineSegment>(mWaypoints.front(), mWaypoints.back());
      auto& line = *pObj;
      line.mPose.setPosition(line.mOrigin);
      setOrientation(line.mPose, line.mDirection);
      return true;
    }
    return false;
  }

  /**
  */
  std::vector<Vec3d> BezierToArcTrajectory::getSamplesPoints() const
  {
    std::vector<Vec3d> result;
    for (const auto& pSeg : mSegments)
    {
      if (const auto* line = dynamic_cast<const LineSegment*>(pSeg.get()))
      {
        result.push_back(line->mOrigin);
      }
      else // nothing else possible
      {
        const auto* circle = dynamic_cast<const CircleSegment*>(pSeg.get());
        result.push_back(circle->interpolate(0).getPosition());
      }
    }
    result.push_back(mWaypoints.back());
    return result;
  }

  /**
  */
  std::vector<Vec3d> BezierToArcTrajectory::getResolutionPoints(double d) const
  {
    std::vector<Vec3d> result;
    for (const auto& pSeg : mSegments)
    {
      const auto N  = std::max(1u, static_cast<unsigned int>(pSeg->length() / d));
      const auto dt = N==0 ? 0.0 : 1.0 / N;
      for (size_t i(0); i<N; ++i)
      {
        const auto t = i*dt;
        result.push_back(pSeg->interpolate(t).getPosition());
      }
    }
    result.push_back(mWaypoints.back());
    return result;
  }

  /**
  */
  std::vector<Vec3d> BezierToArcTrajectory::getDiscretizationPoints(size_t) const
  {
    std::vector<Vec3d> result;
    return result;
  }

  /**
  */
  std::vector<se3::Pose3D> BezierToArcTrajectory::getSamples() const
  {
    std::vector<se3::Pose3D> result;
    for (const auto& seg : mSegments)
    {
      const auto p = seg->interpolate(0);
      result.push_back(p);
    }
    if ( ! mSegments.empty())
      result.push_back(mSegments.back()->interpolate(1));
    return result;
  }

  /** \brief not yet implemented
  */
  std::vector<se3::Pose3D> BezierToArcTrajectory::getResolution(double d) const
  {
    std::vector<se3::Pose3D> result;
    for (const auto& seg : mSegments)
    {
      const auto stepSize = seg->length() / d;
      const auto dt       = 1.0 / stepSize;
      for (double t(0.0); t<1.0; t+=dt)
      {
        const auto p = seg->interpolate(t);
        result.push_back(p);
      }
    }
    if ( ! mSegments.empty())
      result.push_back(mSegments.back()->interpolate(1));
    return result;
  }

  /** \brief not yet implemented
  */
  std::vector<se3::Pose3D> BezierToArcTrajectory::getDiscretization(size_t) const
  {
    std::vector<se3::Pose3D> result;
    return result;
  }
}
}