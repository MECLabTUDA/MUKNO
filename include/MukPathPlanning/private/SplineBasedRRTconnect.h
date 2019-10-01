#ifndef OMPL_GEOMETRIC_PLANNERS_BI_RRT_SPLINE_BASED_2_
#define OMPL_GEOMETRIC_PLANNERS_BI_RRT_SPLINE_BASED_2_

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

#include <Eigen/Core>

namespace ompl
{
  namespace geometric
  {
    /** \brief Implementation of the Spline-Based RRT by Yang et al.

      The proposed RRT does not work properly in our environment.
      The curvature constraint is severe (0.05) compared to the narrow anatomy.
      
    */
    class SplineBasedRRTconnect : public base::Planner
    {
      public:
        SplineBasedRRTconnect(const base::SpaceInformationPtr &si);
        virtual ~SplineBasedRRTconnect();

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
        void    setConeLength(double val)               { mConeLength = val; }
        double  getConeLength()               const     { return mConeLength; }
        void    setConeRadius(double val)               { mConeRadius = val; }
        double  getConeRadius()               const     { return mConeRadius; }
        void    setMaxChildren(size_t val)              { mMaxChildren = val; }
        size_t  getMaxChildren()              const     { return mMaxChildren; }
        size_t  getNumberOfStartTreeRoots()   const;

        template<template<typename T> class NN>
        void setNearestNeighbors()
        {
          mpStartTree.reset(new NN<Motion*>());
          mpGoalTree.reset(new NN<Motion*>());
        }

      protected:
        void freeMemory();
        void freeMotion(Motion* motion) const;
        double distanceFunction(const Motion *a, const Motion *b) const;

      private:
        bool preprocess();
        void postprocess(bool solved, bool approximated);
        bool curvatureSatisfied(const gris::Vec3d& v, const gris::Vec3d& w) const;
        //bool curvatureSatisfied(const Eigen::Vector3d& v, const Eigen::Vector3d& w) const;
        void directionsFromMotions(const Motion* mFrom, const Motion* mTo,  Eigen::Vector3d& vFrom, Eigen::Vector3d& vTo) const;

      private:
        using TreeType = NearestNeighbors<Motion*>;
        /// \brief The result of a call to steer
        enum EnSteerResult
        {          
          enFailed,   /// No extension was possible          
          enAdvanced, /// Progress was made toward extension          
          enSuccess   /// The desired state was reached during extension
        };

        std::vector<Motion*>  selectNearestNeighbors(TreeType& tree, Motion* pMotion);
        std::vector<Motion*>  selectStates(const std::vector<Motion*>& motions);
        Motion*               steer(TreeType& tree, Motion* from, const base::State* to, const base::PlannerTerminationCondition& ptc);
        std::vector<Motion*>  selectStatesFromOtherTree(TreeType& tree, Motion* motion);
        bool                  motionIsValid(const base::State *from, const base::State *to) const;
        bool                  insideCone(const Motion* src, const Motion* query)            const;
        bool                  connectTree(TreeType& treeFrom, Motion* motionFrom, TreeType& treeTo, Motion* motionTo, const base::PlannerTerminationCondition& ptc);

      private:
        std::unique_ptr<TreeType> mpStartTree;
        std::unique_ptr<TreeType> mpGoalTree;
        std::unique_ptr<TreeType> mpConnectionAttempts;
        base::StateSamplerPtr  mpSampler;
        RNG      rng_;

        gris::muk::BezierSpiralInterpolator mSpiralBuilder;
        bool     mVerbose;
        double   mGoalBias;
        double   mKappa;
        double   mStepSize;
        double   mRadiusNearestBall;
        double   mConeLength = 5.0;
        double   mConeRadius = 2.0;
        size_t   mMaxChildren = 20;

        std::pair<Motion*, Motion*> mConnectionPoint;
        Motion* mpLastStartMotion;
        Motion* mpLastGoalMotion;
        gris::muk::PlannerStatistic mStats;
    };

  }
}

#endif
