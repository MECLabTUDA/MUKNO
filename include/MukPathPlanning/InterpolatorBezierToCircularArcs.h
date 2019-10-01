#pragma once
#include "muk_pathplanning_api.h"

#include "MukCommon/IInterpolator.h"

namespace gris
{
  namespace muk
  {
    /** \brief interpolates a series of waypoints for Bezier-Paths according to the transition rule to Circular arcs

      Assumes that the MukPath's member variable hints is filled by the #OptimizerBezierToCircularArcs.
    */
    class MUK_PP_API InterpolatorBezierToCircularArcs : public IInterpolator
    {
      public:
        InterpolatorBezierToCircularArcs();
        virtual ~InterpolatorBezierToCircularArcs();

      public:
        static const char*  s_name()       { return "BezierToCircularArcs-I"; }
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void interpolate()                    override;
        virtual bool isValid()                  const override { return true; }
        virtual result_type getControlPoints()  const override;
        virtual std::vector<bool> validStates() const override;

      protected:
        virtual result_type       getInterpolation(EnInterpolationTypes) const override;
        virtual InterpolatedPoses getInterpolatedPoses(EnInterpolationTypes) const override;

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}

