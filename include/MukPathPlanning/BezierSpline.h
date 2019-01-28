#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/MukVector.h"

#include <array>
#include <vector>

namespace gris
{
  namespace muk
  {

    /** \brief General cubic Bezier Curve
    */
    class MUK_PP_API BezierCurve
    {
      public:
        // Index of sample points in the array
        enum EnIndex
        {
          B0, B1, B2, B3
        };

      public:
        BezierCurve() {}
        BezierCurve(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, const Vec3d& p3);
       ~BezierCurve() {}

      public:
        void          setPoints(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, const Vec3d& p3);
        void          setPoint(int i, const Vec3d& p);
        const Vec3d&  getPoint(int idx) const;

      public:
        double length(size_t fraction = 20)                  const;
        double length(double t, size_t fraction = 20)        const;
        void   uniformSamples(size_t numel, std::vector<double>& output)  const;
        Vec3d  at(double t)                                 const;
        Vec3d  derivativeAt(double t)                       const;

      private:
        std::array<Vec3d, 4> mPoints;
    };

    /** \brief Bezier Spline consisting of two Bezier Spirals

      Spline like in publication: Yang et al., "Spline-Based RRT"
      Each spline consists of two connected Bezier Spirals.
      The Curvature of bothj Spirals at the connection point correspond to each other.
      Curvature at beginning and end of the spline is zero. 
    */
    class MUK_PP_API BezierSpline
    {
      public:
        // Index of B3 equals by definition E3. 
        enum EnIndex
        {
          B0, B1, B2, B3, E2, E1, E0
        };

      public:
        BezierSpline()  {}
        ~BezierSpline() {}

        double length(size_t fraction = 20)           const;
        double length(double t, size_t fraction = 20) const;

        void   uniformSamples(size_t numel,      std::vector<Vec3d>& output)  const;
        void   uniformSamples(double resolution, std::vector<Vec3d>& output)  const;
        Vec3d  at(double t)           const;        
        const Vec3d& sample(int index)  const { return samples[index]; }
              Vec3d& sample(int index)        { return samples[index]; }
        Vec3d  derivativeAt(double t) const;

      private:
        Vec3d  at(int t) const;

      public:
        std::array<Vec3d, 7> samples;
    };
    
  }
}
