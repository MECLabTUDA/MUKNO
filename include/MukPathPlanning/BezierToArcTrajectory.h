#pragma once
#include "muk_pathplanning_api.h"

#include "MukCommon/LieGroupSE3.h"
#include "MukCommon/geometry.h"

#include <memory>

namespace gris
{
  namespace muk
  {
    /** \brief Implementation of a trajectory that follows circular arcs of varying curvature.

      Each segment is either a straight line, a min circle or a max circle.
    */
    class MUK_PP_API BezierToArcTrajectory
    {
      public:
        BezierToArcTrajectory() {};
        ~BezierToArcTrajectory();

        BezierToArcTrajectory(const BezierToArcTrajectory& o) = delete;
        BezierToArcTrajectory(BezierToArcTrajectory&& o)      = delete;
        BezierToArcTrajectory& operator=(const BezierToArcTrajectory& o) = delete;
        BezierToArcTrajectory& operator=(BezierToArcTrajectory&& o)      = delete;

      public:
        using Pose3D = se3::Pose3D;

      public:
        void setKappa(double val)  { mKappa  = val; }
        void setWaypoints(const std::vector<Vec3d>& waypoints) { mWaypoints = waypoints; }
        void setHints    (const std::vector<int>& hints)       { mHints = hints; }
        void compute();

      public:
        const std::vector<Vec3d>&   getWaypoints()                    const { return mWaypoints; }
        std::vector<Vec3d>          getSamplesPoints()                const;
        std::vector<Vec3d>          getResolutionPoints(double d)     const;
        std::vector<Vec3d>          getDiscretizationPoints(size_t)   const;
        std::vector<Pose3D>         getSamples()                      const;
        std::vector<Pose3D>         getResolution(double d)           const;
        std::vector<Pose3D>         getDiscretization(size_t)         const;

      private:
        bool        isEasyCase();
        se3::Pose3D getPose(const Vec3d& B0, const Vec3d& zAxis, const Vec3d& yAxis);

      private:
        /**
        */
        struct MUK_PP_API Segment
        {
          virtual ~Segment();

          virtual void   computePush() = 0;
          virtual void   computeRotation(const Pose3D& poseTarget);

          virtual double length()              const = 0;
          virtual Pose3D interpolate(double t) const = 0;

          Pose3D           mPose;
          double           mPhi = 0.0;
          se3::TwistVector mTwist;
        };

        /**
        */
        struct MUK_PP_API LineSegment : public Segment
        {
          LineSegment() {}
          LineSegment(const Vec3d& start, const Vec3d& end)
          {
            mOrigin     = start;
            mDirection  = (end - start);
            mLength     = mDirection.norm();
            mDirection /= mLength;
          }
          virtual ~LineSegment();

          virtual void   computePush();

          virtual double length()              const override { return mLength; }
          virtual Pose3D interpolate(double t) const override;

          Vec3d   mOrigin;
          Vec3d   mDirection;
          double  mLength;
        };

        /**
        */
        struct MUK_PP_API CircleSegment : public Segment
        {
          virtual ~CircleSegment();

          virtual void   computePush();

          virtual double length()              const override { return mCircle.getRadius() * mMaxAngle; }
          virtual Pose3D interpolate(double t) const override;

          Circle3D mCircle;
          double   mMaxAngle;
        };

      private:
        std::vector<std::shared_ptr<Segment>> mSegments;  // Circular Arc representation
        std::vector<Vec3d>   mWaypoints; // Bézier spline waypoints
        std::vector<int>     mHints;     // quickfix: hints if min or max circle
        double mKappa;
        double mRadius;
    };
  }
}