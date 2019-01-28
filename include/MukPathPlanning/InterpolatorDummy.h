#pragma once

#include "IInterpolator.h"

namespace gris
{
  namespace muk
  {

    class MUK_PP_API InterpolatorDummy : public IInterpolator
    {
      public:
        InterpolatorDummy();
        virtual ~InterpolatorDummy();

      public:        
        static  const char*  s_name()       { return "Dummy-Interpolator"; }
        virtual const char*  name()  const  { return s_name(); }

      private:
        InterpolatorDummy(const InterpolatorDummy& o);

      public:
        virtual bool isValid();
        virtual bool isValid(const MukPath& path);

      public:        
        virtual void interpolate(const MukPath& path);
        virtual result_type getControlPoints();
        virtual result_type getInterpolation();
        virtual result_type getInterpolation(size_t n);
        virtual result_type getInterpolation(double resolution);
        virtual result_type getInterpolation(const result_type& input, size_t n);
        virtual result_type getInterpolation(const result_type& input, double resolution);
    };

  }
}