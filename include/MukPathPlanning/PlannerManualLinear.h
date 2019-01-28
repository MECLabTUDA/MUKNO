#pragma once

#include "muk_pathplanning_api.h"

#include "MukCommon/IPathPlanner.h"

#include <memory>
#include <vector>

namespace gris
{
  namespace muk
  {

    /** \brief Interface for the path-planning procedure

    Implemented classes will internally define and configure an ompl::base::Planner
    */
    class MUK_PP_API PlannerManualLinear : public IPathPlanner
    {
      public:
        PlannerManualLinear();
        virtual ~PlannerManualLinear();

        static  const char* s_name()        { return "ManualPlanner"; }
        virtual const char* name()   const  { return s_name(); }

      public:
        virtual void initialize() {}
        virtual bool calculate();
        virtual void update() {}
        virtual void clearSearch() { mPaths.clear(); }
        virtual size_t availablePaths() const;

      public:
        virtual MukPath        extractPath(size_t idx=0) const;
        virtual MukPathGraph   extractRoadmap()       const;

      private:
        std::vector<MukPath> mPaths;
    };

  }
}

