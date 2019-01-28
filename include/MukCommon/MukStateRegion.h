#pragma once

#include "muk_common_api.h"

#include "IStateRegion.h"

namespace gris
{
  namespace muk
  {
    /** \brief Abstraction for a Goal/Start-State

      atm, only this sphere like region is implemented
    */
    class MUK_COMMON_API MukStateRegion : public IStateRegion
    {
      public:
        MukStateRegion();
        virtual ~MukStateRegion() = default;

        static  const char* s_name()     { return "StateRegionSphere";  }
        virtual const char* name() const { return s_name(); }

      public:
        void setCenter(const MukState& s) { mCenter = s; }
        void setRadius (double val)       { mRadius = val; }
        void setPhi (double val);
        void setResolution (unsigned int val)   { mResolution = val; }
                
        double          getRadius() const { return mRadius; }
        double          getPhi()  const { return mPhi; }
        unsigned int    getResolution() const { return mResolution; }

        const MukState& getCenter() const { return mCenter; }
        
      public:
        virtual std::vector<MukState> getStates() const;
        
      private:
        MukState mCenter;
        double mRadius;
        double mPhi; // latitude  [0, pi], use mathematic convention, not physics (phi and theta switched) !!!
        unsigned int mResolution;

        Use_Muk_Boost_Serialization
    };
  }
}
