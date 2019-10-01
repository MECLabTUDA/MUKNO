#pragma once

#include "muk_common_api.h"

#include "IStateRegion.h"

namespace gris
{
  namespace muk
  {
    /** \brief A state region representing a position and direction with additional deviations.

      Atm, the radius is not used and #getStates returns only states at the center position.
      The #resolution parameter is used to sample from [0,phi] in polar coordinates, making different directions possible.
    */
    class MUK_COMMON_API MukStateRegion : public IStateRegion
    {
      public:
        MukStateRegion();
        virtual ~MukStateRegion() = default;

      public:
        static  const char* s_name()     { return "StateRegionSphere";  }
        virtual const char* name() const { return s_name(); }

      public:
        virtual std::vector<MukState>         getStates() const;
        virtual std::unique_ptr<IStateRegion> clone()     const override;

      public:
        void            setCenter(const MukState& s) { mCenter = s; }
        void            setRadius (double val)       { mRadius = val; }
        void            setPhi (double val);
        void            setResolution (unsigned int val)   { mResolution = val; }
                
        double          getRadius() const { return mRadius; }
        double          getPhi()  const { return mPhi; }
        unsigned int    getResolution() const { return mResolution; }
        const MukState& getCenter() const { return mCenter; }
        
      private:
        MukState mCenter;
        double mRadius;
        double mPhi; // latitude  [0, pi], use mathematic convention, not physics (phi and theta switched) !!!
        unsigned int mResolution;

        Use_Muk_Boost_Serialization
    };
  }
}
