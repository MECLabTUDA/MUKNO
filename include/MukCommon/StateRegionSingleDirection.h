#pragma once

#include "IStateRegion.h"

namespace gris
{
  namespace muk
  {
    /** \brief A state region with a number of positions with fixed direction.
    */
    class MUK_COMMON_API StateRegionSingleDirection : public IStateRegion
    {
      public:
        StateRegionSingleDirection() = default;
        StateRegionSingleDirection(const StateRegionSingleDirection&) = default;
        StateRegionSingleDirection& operator=(const StateRegionSingleDirection&) = default;
        StateRegionSingleDirection(StateRegionSingleDirection&&) = default;
        StateRegionSingleDirection& operator=(StateRegionSingleDirection&&) = default;
        virtual ~StateRegionSingleDirection();

      public:
        static  const char* s_name()              { return "StateRegionSingleDirection"; }
        virtual const char* name() const override { return s_name(); }

      public:
        virtual std::vector<MukState>         getStates() const override;
        virtual std::unique_ptr<IStateRegion> clone()     const override;

      public:
        void         setPoints(const std::vector<Vec3d>& points) { mPoints = points; }
        void         setDirection(const Vec3d& p)         { mDirection = p; }
        const Vec3d& getDirection()                const  { return mDirection; }

      private:
        std::vector<Vec3d> mPoints;
        Vec3d mDirection;
        Use_Muk_Boost_Serialization
    };
  }
}
