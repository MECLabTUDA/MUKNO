#pragma once
#include "OmplPlanner.h"

#include <memory>

namespace ompl
{
  namespace geometric
  {
    class SplineBasedRRT;
  }
}

namespace gris
{
  namespace muk
  {
    /** \brief Wrapper for the Implementation of the Spline-Based RRT

      Private implementation implements the actual Spline-Based RRT dependend of the Open Motion Planning Library.
      This class wraps OMPL. 
      Initializing creates the ompl configuration
      Updating takes the member variables and passes them to the actual implementation.
    */
    class MUK_PP_API PlannerSplineBasedRRT : public OmplPlanner
    {
      public:
        PlannerSplineBasedRRT();
        ~PlannerSplineBasedRRT();

        static  const char* s_name()      { return "Spline-Based-RRT"; }
        virtual const char* name() const  { return s_name(); }

      public:
        virtual void initialize();
        virtual void update();
        virtual bool calculate();
        virtual void clearSearch();
        virtual size_t availablePaths() const;

        virtual MukPath        extractPath(size_t idx=0)  const;
        virtual MukPathGraph   extractRoadmap()           const;

      public:
        void    setGoalBias(double d)                 { mGoalBias = d;}
        double  getGoalBias()                   const { return mGoalBias; }
        void    setNearestBallRadius(double d)        { mNearestBallRadius = d;}
        double  getNearestBallRadius()          const { return mNearestBallRadius; }
        void    setSwitchStartAndGoal(bool d);
        bool    getSwitchStartAndGoal()         const { return mSwitchStartAndGoal; }
        
      private:
        std::unique_ptr<ompl::geometric::SplineBasedRRT> mpPlanner;
        double mGoalBias;
        double mNearestBallRadius;
        bool   mSwitchStartAndGoal; /// This planner can plan from start to goal or from goal to start. It defaults to planning from goal to start.
    };

  }
}