#pragma once
#include "muk_pathplanning_api.h"

#include "MukCommon/IInterpolator.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_PP_API InterpolatorLinear : public IInterpolator
    {
      public:
        InterpolatorLinear() {}
        virtual ~InterpolatorLinear() {}
      
      public:
        static  const char* s_name()        { return "Linear-Interpolator"; }
        virtual const char* name() const    { return s_name(); }
                
        virtual void interpolate();

      public:
        virtual bool isValid() const;

      public:
        using IInterpolator::getInterpolation;
        virtual result_type       getControlPoints()  const;        
        virtual std::vector<bool> validStates()       const;

      protected:
        virtual result_type getInterpolation(EnInterpolationTypes) const;

      private:
        mutable size_t mNumelResult;
    };
  }
}
