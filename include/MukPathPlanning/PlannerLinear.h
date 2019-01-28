#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/IPathPlanner.h"

namespace gris
{
  namespace muk
  {
    /** \brief computes best paths from start region to goal region

    */
    class MUK_PP_API PlannerLinear : public IPathPlanner
    {
      public:
        PlannerLinear();
        virtual ~PlannerLinear();

        static  const char* s_name()        { return "Linear-Planner"; }
        virtual const char* name()   const  { return s_name(); }

      public:
        void   setMaxDistToGoal(double d)       { mMaxDistToGoal = d; }
        double getMaxDistToGoal()         const { return mMaxDistToGoal; };
        void   setResolution(double d)          { mResolution = d; }
        double getResolution()            const { return mResolution; };
        
      public:
        virtual void  initialize() {}
        virtual bool  calculate();
        virtual void  update() {}
        virtual void  clearSearch() { mFoundPaths.clear(); }
        virtual size_t availablePaths() const;

      public:
        virtual MukPath        extractPath(size_t idx=0) const;
        virtual MukPathGraph   extractRoadmap()       const;

      private:
        double mResolution;
        double mMaxDistToGoal;
        std::vector<MukPath> mFoundPaths;
    };

  }
}