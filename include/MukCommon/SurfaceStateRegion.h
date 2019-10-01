#pragma once
#include "IStateRegion.h"

namespace gris
{
  namespace muk
  {
    /** \brief A state region with a number of positions with directions pointing towards a common "ancor".
      
      The boolean #mPointsTowardsAncor can be used to define if directions should point towards or away from the ancor.
    */
    class MUK_COMMON_API SurfaceStateRegion : public IStateRegion
    {
      public:
        SurfaceStateRegion();
        virtual ~SurfaceStateRegion() = default;

      public:
        static  const char* s_name()              { return "SurfaceStateRegion"; }
        virtual const char* name() const override { return s_name(); }

      public:
        virtual std::vector<MukState>         getStates() const override;
        virtual std::unique_ptr<IStateRegion> clone()     const override;

      public:
        void   setPoints(const std::vector<Vec3d>&& p)      { mSurfacePoints = std::move(p); }
        void   setDirectionAncor(const Vec3d& p)            { mDirectionAncor = p; }
        void   setPointTowardsAncor(bool b)                 { mPointsTowardsAncor = b;  }

        const MukState&           getCenter()         const;
        const std::vector<Vec3d>& getPoints()         const { return mSurfacePoints; }
        const Vec3d&              getDirectionAncor() const { return mDirectionAncor; }

      private:
        std::vector<Vec3d> mSurfacePoints;
        Vec3d mDirectionAncor;
        bool  mPointsTowardsAncor;
        Use_Muk_Boost_Serialization
    };
  }
}
