#pragma once
#include "OmplPlanner.h"

#include <memory>

namespace ompl
{
  namespace geometric
  {
    class SplineBasedRRTconnect;
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
    class MUK_PP_API PlannerSplineBasedRRTconnect : public OmplPlanner
    {
      public:
        PlannerSplineBasedRRTconnect();
        ~PlannerSplineBasedRRTconnect();

        static  const char* s_name()      { return "Spline-Based-RRT-Connect"; }
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
        void    setConeLength(double d)               { mConeLength= d;}
        double  getConeLength()                 const { return mConeLength; }
        void    setConeRadius(double d)               { mConeRadius= d;}
        double  getConeRadius()                 const { return mConeRadius; }

      private:
        std::unique_ptr<ompl::geometric::SplineBasedRRTconnect> mpPlanner;
        double mGoalBias;
        double mNearestBallRadius;
        double mConeLength;
        double mConeRadius;
        size_t mMaxChildren;
    };
  }
}