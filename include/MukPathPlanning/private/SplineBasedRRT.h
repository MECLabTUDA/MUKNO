#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_SPLINE_BASED
#define OMPL_GEOMETRIC_PLANNERS_RRT_SPLINE_BASED

#include "BezierSpiralInterpolator.h"
#include "private/Motion.h"
#include "private/PlannerStatistic.h"

#pragma warning (push)
#pragma warning( disable: 4267 ) // unsigned int vs size_t
#pragma warning( disable: 4800 ) // forcing value to bool
#pragma warning( disable: 4996 ) // function (localtime) may be unsafe
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#pragma warning (pop)

namespace ompl
{
  namespace geometric
  {
    /** \brief Implementation of the SplineBased-RRT by Yang et al.

      The proposed RRT does not work in our environment.
      The actual algorithm works only if the curvature corresponds to the environment in a nice way....
      This implementation uses a similar approach to 
        Sertac Karaman, Sampling-Based Algorithms for Optimal Motion Planning
        optimal RRT (RRT*)
    */
    class SplineBasedRRT : public base::Planner
    {
      public:
        SplineBasedRRT(const base::SpaceInformationPtr &si);
        virtual ~SplineBasedRRT();

      public:
        virtual void setup();
        virtual void clear();
        virtual void getPlannerData(base::PlannerData &data) const;
        using   Planner::solve;
        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);        
        
        void restartSearch();

        const gris::muk::PlannerStatistic& getStats() const { return mStats; };

      public:
        void    setVerbose(bool val)                    { mVerbose = val; }
        bool    getVerbose()                  const     { return mVerbose; }
        void    setKappa (double val)                   { mKappa = val; mSpiralBuilder.setKappa(val); }
        double  getKappa ()                   const     { return mKappa; }
        void    setStepSize (double val);
        double  getStepSize ()                const     { return mStepSize; }
        void    setGoalBias (double goalBias)           { mGoalBias = goalBias; }
        double  getGoalBias ()                const     { return mGoalBias; }
        void    setNearestBallRadius (double val)       { mRadiusNearestBall = val; }
        double  getNearestBallRadius()        const     { return mRadiusNearestBall; }
        
        template<template<typename T> class NN>
        void setNearestNeighbors()
        {
          mpNN.reset(new NN<Motion*>());
        }

      protected:
        void freeMemory();
        void freeMotion(Motion* motion) const;

        double distanceFunction(const Motion *a, const Motion *b) const;

      private:
        bool preProcess();
        void postProcess(bool solved, bool approximated, double distance);
        std::vector<Motion*> selectNearestNeighbors(Motion* pMotion);
        std::vector<Motion*> selectStates(const std::vector<Motion*>& motions);
        void  steer(Motion* from, const base::State* to, double& approxdif, Motion* approxSolution, const base::PlannerTerminationCondition &);
        bool  motionIsValid(const base::State *from, const base::State *to) const;
        
      private:
        std::unique_ptr< NearestNeighbors<Motion*> > mpStartGraph;
        base::StateSamplerPtr  mpSampler;
        RNG      rng_;

        gris::muk::BezierSpiralInterpolator mSpiralBuilder;
        bool     mVerbose;
        double   mGoalBias;
        double   mKappa;
        double   mStepSize;
        double   mRadiusNearestBall;
        
        Motion*  mpLastGoalMotion;
        Motion*  mpFoundSolution;
        gris::muk::PlannerStatistic mStats;
    };

  }
}

#endif