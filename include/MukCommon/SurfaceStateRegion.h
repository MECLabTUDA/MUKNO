#pragma once
#include "IStateRegion.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_COMMON_API SurfaceStateRegion : public IStateRegion
    {
      public:
        SurfaceStateRegion();
        virtual ~SurfaceStateRegion() = default;

        static  const char* s_name()     { return "SurfaceStateRegion"; }
        virtual const char* name() const { return s_name(); }

      public:
        void   setPoints(const std::vector<Vec3d>&& p)  { mSurfacePoints = std::move(p); }
        void   setDirectionAncor(const Vec3d& p)        { mDirectionAncor = p; }
        void   setPointTowardsAncor(bool b)             { mPointsTowardsAncor = b;  }

        const MukState& getCenter() const;

        const std::vector<Vec3d>& getPoints()         const { return mSurfacePoints; }
        const Vec3d&              getDirectionAncor() const { return mDirectionAncor; }

      public:
        virtual std::vector<MukState> getStates() const;

      private:
        std::vector<Vec3d> mSurfacePoints;
        Vec3d mDirectionAncor;
        bool  mPointsTowardsAncor;

        Use_Muk_Boost_Serialization
    };
  }
}
