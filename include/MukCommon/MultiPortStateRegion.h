#pragma once

#include "muk_common_api.h"

#include "IStateRegion.h"

namespace gris
{
  namespace muk
  {
    /** \brief A state region representing multiple position and direction with additional deviations (experimental).

      Atm, the radius is not used and #getStates returns only states at given positions.
      The #resolution parameter is used to sample from [0,phi] in polar coordinates, making different directions possible.
    */
    class MUK_COMMON_API MultiPortStateRegion : public IStateRegion
    {
      public:
        MultiPortStateRegion();
        virtual ~MultiPortStateRegion() = default;

      public:
        static  const char* s_name()              { return "MultiPortStateRegion";  }
        virtual const char* name() const override { return s_name(); }

      public:
        virtual std::vector<MukState>         getStates() const override;
        virtual std::unique_ptr<IStateRegion> clone()     const override;

      public:
        void            setNumberOfStates(size_t n)       { mStates.resize(n); }
        void            setState  (size_t idx, const MukState& s);
        void            setRadius (double val)            { mRadius = val; }
        void            setPhi    (double val);
        void            setResolution (unsigned int val)  { mResolution = val; }

        size_t          getNumberOfStates()         const { return mStates.size(); }
        double          getRadius()                 const { return mRadius; }
        double          getPhi()                    const { return mPhi; }
        unsigned int    getResolution()             const { return mResolution; }
        const MukState& getCenter()                 const { return mStates.front(); }

      private:
        std::vector<MukState> mStates;
        double mRadius;
        double mPhi; // latitude  [0, pi], use mathematic convention, not physics (phi and theta switched) !!!
        unsigned int mResolution;

        Use_Muk_Boost_Serialization
    };
  }
}
