#pragma once

#include "muk_common_api.h"

#include "IStateRegion.h"

namespace gris
{
  namespace muk
  {
    /** \brief A state region with a number of positions on a plane with directions pointing towards a common "ancor".

      States on the plane are restricted to lie within a polygon on the plane. 
      A regular rectangular grid on the plane is used to create states in #getStates.
      A resolution parameter determines the grid size.
    */
    class MUK_COMMON_API PlaneStateRegion : public IStateRegion
    {
      public:
        PlaneStateRegion();
        virtual ~PlaneStateRegion() = default;

      public:
        static  const char* s_name()              { return "PlaneStateRegion";  }
        virtual const char* name() const override { return s_name(); }

      public:
        virtual std::vector<MukState>         getStates() const override;
        virtual std::unique_ptr<IStateRegion> clone()     const override;

      public:
        void   setOrigin(const Vec3d& p)                      { mOrigin = p; }
        void   setNormal(const Vec3d& p)                      { mNormal = p; }
        void   setDirectionAncor(const Vec3d& p)              { mDirectionAncor = p; }
        void   setBoundingPolygon(std::vector<Vec3d> poly)    { mBoundingPolygon = std::move(poly); }
        void   setResolution (double val)                     { mResolution = val; }
        void   setPointsTowardsAncor(bool b)                  { mPointsTowardsAncor = b; }

        const Vec3d& getOrigin()                        const { return mOrigin; }
        const Vec3d& getNormal()                        const { return mNormal; }
        const Vec3d& getDirection()                     const { return mDirectionAncor; }
        const std::vector<Vec3d>& getBoundingPolygon()  const { return mBoundingPolygon; }
        const MukState& getCenter()                     const;

      private:
        Vec3d mOrigin;
        Vec3d mNormal;
        Vec3d mDirectionAncor;
        std::vector<Vec3d> mBoundingPolygon;
        double mResolution;
        bool   mPointsTowardsAncor;

        Use_Muk_Boost_Serialization
    };
  }
}
