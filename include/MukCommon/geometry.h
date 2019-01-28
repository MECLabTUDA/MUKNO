#pragma once

#include "muk_common_api.h"
#include "MukVector.h"

#include <vector>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_COMMON_API Sphere3D
    {
      public:
        Sphere3D()  {}
        ~Sphere3D() {}

      public:
        void setRadius(double r)        { mRadius = r; }
        void setCenter(const Vec3d& c)  { mCenter = c; }
        double getRadius()       const { return mRadius; }
        const Vec3d& getCenter() const { return mCenter; }

      private:
        Vec3d  mCenter;
        double mRadius;
    };

    /**
    */
    class MUK_COMMON_API Plane3D
    {
      public:
      Plane3D()  {}
      ~Plane3D() {}

      public:
      void setNormal(const Vec3d& n)  { mNormal = n; }
      void setOrigin(const Vec3d& o)  { mOrigin = o; }
      const Vec3d& getNormal() const  { return mNormal; }
      const Vec3d& getOrigin() const  { return mOrigin; }

      private:
      Vec3d  mOrigin;
      Vec3d  mNormal;
    };

    /*class MUK_COMMON_API Plane2D
    {
      public:
      Plane2D()  {}
      ~Plane2D() {}

      public:
      void setNormal(const Vec2d& n)  { mNormal = n; }
      void setOrigin(const Vec2d& o)  { mOrigin = o; }
      const Vec2d& getNormal() const  { return mNormal; }
      const Vec2d& getOrigin() const  { return mOrigin; }

      private:
      Vec2d  mOrigin;
      Vec2d  mNormal;
    };*/
    
    /**
    */
    class MUK_COMMON_API Circle2D
    {
      public:
        Circle2D()  {}
        ~Circle2D() {}

      public:
        void setRadius(double r)        { mRadius = r; }
        void setCenter(const Vec2d& c)  { mCenter = c; }
        double getRadius()       const { return mRadius; }
        const Vec2d& getCenter() const { return mCenter; }

      public:
        bool isInside(const Vec2d& p);

      private:
        Vec2d  mCenter;
        double mRadius;
    };

    /** 
    */
    class MUK_COMMON_API Circle3D
    {
      public:
        Circle3D()  {}
        ~Circle3D() {}

      public:
        void setRadius(double r)        { mRadius = r; mInitialized = false; }
        void setCenter(const Vec3d& c)  { mCenter = c; mInitialized = false; }
        void setNormal(const Vec3d& n)  { mNormal = n; mInitialized = false; }
        void setXAxis  (const Vec3d& v);
        double getRadius()       const { return mRadius; }
        const Vec3d& getCenter() const { return mCenter; }
        const Vec3d& getNormal() const { return mNormal; }

        Vec3d interpolate(double t) const;
        Vec3d direction(double t)   const;
        double angle(const Vec3d& v) const;

      private:
        void initialize() const;

      private:
        Vec3d  mCenter;
        Vec3d  mNormal;
        double mRadius;

        mutable Vec3d  mxAxis;
        mutable Vec3d  myAxis;        
        mutable bool mInitialized;
    };

    /**
    */
    class MUK_COMMON_API Line3D
    {
      public:
        struct Query
        {
          enum EnIntersectionType
          {
            enIntersects = 0,
            enSkewLines,
            enParallel,
          };

          int   intersectionType;
          Vec3d intersectionPoint1;
          Vec3d intersectionPoint2;
        };

      public:
        Line3D();
        Line3D(const Vec3d& origin, const Vec3d& direction);
        ~Line3D();

      public:
        void          setOrigin(const Vec3d& p)    { mOrigin = p; }
        void          setDirection(const Vec3d& p) { mDirection = p; }
        const Vec3d&  getOrigin()    const { return mOrigin; }
        const Vec3d&  getDirection() const { return mDirection; }

      public:
        void intersects(const Line3D& line, Query& query);

      private:
        Vec3d mOrigin;
        Vec3d mDirection;
    };
        
    MUK_COMMON_API double wideAngle(const Vec3d& v, const Vec3d& w);
    MUK_COMMON_API double angleIn2D(const Vec3d& v, const Vec3d& w);

    MUK_COMMON_API Vec3d rotateOntoVector(const Vec3d& from, const Vec3d& to);

    MUK_COMMON_API void fitCircle(const std::vector<Vec2d>& input, gris::muk::Circle2D& output);
    MUK_COMMON_API void fitSphere(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, Sphere3D& output);
    MUK_COMMON_API void fitSphere(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, const Vec3d& p4, Sphere3D& output);
    MUK_COMMON_API void fitPlane (const VecN3d& points, Plane3D& plane);

    MUK_COMMON_API void circleAroundLine(const Vec3d& center, const Vec3d& tangent, double r, std::vector<Vec3d>& samples);
    MUK_COMMON_API void inscribedPolygon(const std::vector<Vec3d>& input, std::vector<Vec3d>& output);
    
    MUK_COMMON_API void approxInscribedCircle(const std::vector<Vec3d>& input, Vec3d& output);

    MUK_COMMON_API void maxCurvatureFromMaxAngle(double maxAngle, double length, double& output);

  }
}
