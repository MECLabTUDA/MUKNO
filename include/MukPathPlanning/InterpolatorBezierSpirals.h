#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/IInterpolator.h"

namespace gris
{
  namespace muk
  {
    /** \brief Interpolates between two MukStates via Bezier Splines.

      Wrapper of BezierSpiral3DInterpolator            
    */
    class MUK_PP_API InterpolatorBezierSpirals : public IInterpolator
    {
        
      public:        
        InterpolatorBezierSpirals();
        virtual ~InterpolatorBezierSpirals();

      public:
        static  const char*   s_name()          { return "Bezier-Spiral-Interpolator"; }
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

      public:
        virtual result_type getInterpolation(EnInterpolationTypes) const;

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}

