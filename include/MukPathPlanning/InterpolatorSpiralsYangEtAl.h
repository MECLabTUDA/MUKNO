#pragma once
#include "muk_pathplanning_api.h"

#include "MukCommon/IInterpolator.h"

namespace gris
{
  namespace muk
  {                    
    /** \brief Wrapper for the interpolation Technique of Yang et al.
    */
    class MUK_PP_API InterpolatorSpiralsYangEtAl : public IInterpolator
    {
      public:        
        InterpolatorSpiralsYangEtAl();
        virtual ~InterpolatorSpiralsYangEtAl();

      public:
        static  const char*   s_name()          { return "Yang-Spiral-Interpolator"; }
        virtual const char*   name()    const   { return s_name(); }

      public:
        virtual void    setKappa(double kappa);
        virtual double  getKappa() const;

      public:
        virtual bool isValid()     const;
        virtual void interpolate();

      public:
        using IInterpolator::getInterpolation;
        virtual result_type getControlPoints()            const;        
        virtual std::vector<bool> validStates() const;

      private:
        virtual result_type getInterpolation(EnInterpolationTypes) const;

      private:
        struct Impl;
        std::unique_ptr<Impl> mp; 
    };
  }
}

