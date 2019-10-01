#pragma once
#include "opt/Optimizer.h"

#include "opt/ConvexConstraints.h"
#include "opt/ConvexObjective.h"


namespace gris
{
  namespace opt
  {
    /*
    * Alternates between convexifying objectives and constraints and then solving convex subproblem
    * Uses a merit function to decide whether or not to accept the step
    * merit function = objective + merit_err_coeff * | constraint_error |
    * Note: sometimes the convexified objectives and constraints lead to an infeasible subproblem
    * In that case, you should turn them into penalties and solve that problem
    * (todo: implement penalty-based sqp that gracefully handles infeasible constraints)
    */
    class BasicTrustRegion : public Optimizer 
    {
      public:
        BasicTrustRegion();
        BasicTrustRegion(OptimizationProblemPtr prob);

      public:
        void              setProblem(OptimizationProblemPtr prob);
        EnOptimizerResult optimize();

      public:
        bool   getVerbose()                         const { return mVerbose; }
        double getConstraintTolerance()             const { return mConstraintTolerance; }
        double getInitialMeritErrorCoeff()          const { return mInitialMeritErrorCoeff; }
        double getInitialTrustRegionSize()          const { return mInitialTrustRegionSize; }
        double getMaxTrustRegionSize()              const { return mMaxTrustRegionSize; }
        size_t getPenaltyIterationLimit()           const { return mIterationLimitPenalty; }
        double getMeritIncreaseRation()             const { return mMeritIncreaseRatio; }
        double getMeritImproveRatioThreshold()      const { return mMeritImproveRatioThreshold; }
        size_t getSCOIterationLimit()               const { return mIterationLimitSCO; }
        size_t getTrustRegionIterationLimit()       const { return mTrustRegionIterationLimit; }
        double getMinTrustRegionSize()              const { return mMinTrustRegionSize; }
        double getTrustRegionScaleDownFactor()      const { return mTrustRegionScaleDownFactor; }
        double getTrustRegionScaleUpFactor()        const { return mTrustRegionScaleUpFactor; }
        double getModelImprovementThreshold()       const { return mModelImprovementThreshold; }
        double getModelImprovementRatioThreshold()  const { return mModelImprovementRatioThreshold; }

        void   setVerbose(bool val);
        void   setConstraintTolerance(double val)                   { mConstraintTolerance = val; }
        void   setInitialMeritErrorCoeff(double val)                { mInitialMeritErrorCoeff = val; }
        void   setInitialTrustRegionSize(double val)                { mInitialTrustRegionSize = val; }
        void   setPenaltyIterationLimit(size_t val)                 { mIterationLimitPenalty = val; }
        void   setMeritIncreaseRation(double val)                   { mMeritIncreaseRatio = val; }
        void   setMeritImproveRatioThreshold(double val)            { mMeritImproveRatioThreshold = val; }
        void   setSCOIterationLimit(size_t val)                     { mIterationLimitSCO = val; }
        size_t setTrustRegionIterationLimit(size_t val)             { mTrustRegionIterationLimit = val; }
        void   setMinTrustRegionSize(double val)                    { mMinTrustRegionSize = val; }
        void   setMaxTrustRegionSize(double val)                    { mMaxTrustRegionSize = val; }
        void   setTrustRegionScaleDownFactor(double val)            { mTrustRegionScaleDownFactor = val; }
        void   setTrustRegionScaleUpFactor(double val)              { mTrustRegionScaleUpFactor = val; }
        void   setModelImprovementThreshold(double val)             { mModelImprovementThreshold = val; }
        void   setModelImprovementRatioThreshold(double val)        { mModelImprovementRatioThreshold = val; }

      private:
        /** \brief The internal state codes results during the different inner loops
        */
        enum EnInternalState
        {
          enInitialized,
          enTrustRegionFailed,
          enSQPFailed,
          enPenaltyRunFailed,
          enConstraintsSatisfied,
          enConstraintsViolated,
          enCheckIfConverged,
        };

        /** \brief holds relevant data that is needed in between loops
        */
        struct Worker
        {
          EnOptimizerResult                 globalStatus;
          EnInternalState                   loopState;
          std::vector<ConstraintPtr>        constraints;
          size_t iterTrustBox;
          size_t iterPenalty;
          size_t iterConvexify;
          std::vector<ConvexObjectivePtr>   convexifiedCosts;
          std::vector<ConvexConstraintsPtr> convexifiedConstraints;
        };

        void penaltyRuns (std::vector<double>& estimate, Worker& w); /// outer loop
        void sqpRuns     (std::vector<double>& estimate, Worker& w); /// middle loop
        void trustBoxRuns(std::vector<double>& estimate, Worker& w); /// inner loop

      private:
        void restrictBoundsToTrustRegion(const std::vector<double>& x);
        void initParameters();

        std::vector<ConvexObjectivePtr>   convexifyCosts(const std::vector<CostPtr>& costs, const std::vector<double>& estimate)              const;
        std::vector<ConvexConstraintsPtr> convexifyConstraints(const std::vector<ConstraintPtr>& costs, const std::vector<double>& estimate)  const;
        std::vector<ConvexObjectivePtr>   convertConstraintToCosts(const std::vector<ConvexConstraintsPtr>& costs)                            const;
        std::vector<double>               evaluateCosts         (const std::vector<CostPtr>& costs,      const std::vector<double>& x)        const;
        std::vector<double>               evaluateCntViols      (const std::vector<ConstraintPtr>& cnts, const std::vector<double>& x)        const;
        std::vector<double>               evaluateModelCosts    (const std::vector<ConvexObjectivePtr>& costs,  const std::vector<double>& x) const;
        std::vector<double>               evaluateModelCntViols (const std::vector<ConvexConstraintsPtr>& cnts, const std::vector<double>& x) const;

      private:
        ConvexOptimizationSolverPtr mpSolver;
        bool  mVerbose = true;
        // fixed member varaibles during solving
        double mConstraintTolerance;        // after convergence of penalty subproblem, if constraint violation is less than this, we're done
        double mInitialMeritErrorCoeff;     // initial penalty coefficient
        double mInitialTrustRegionSize;     // initial size of trust region (component-wise)

        size_t mIterationLimitPenalty;      // number of times that we jack up penalty coefficient
        double mMeritIncreaseRatio;         // ratio that we increase coeff each time
        double mMeritImproveRatioThreshold; // minimum ratio true_improve/approx_improve to accept step

        size_t mIterationLimitSCO;          // maximal number of iterations during a sco run
        double mMinTrustRegionSize;         // if trust region gets any smaller, exit and report convergence
        double mMaxTrustRegionSize;         // do not allow larger trust regions.
        double mTrustRegionScaleDownFactor; // if improvement is less than improve_ratio_threshold, shrink trust region by this ratio
        double mTrustRegionScaleUpFactor;   // if improvement is less than improve_ratio_threshold, expand trust region by this ratio
        size_t mTrustRegionIterationLimit;  // used to enforce regular convexification 
        double mModelImprovementThreshold;  // if model improves less than this, exit and report convergence
        double mModelImprovementRatioThreshold; // if model improves less than this, exit and report convergence

        // working variables durign solving
        double mCurrentTrustRegionSize;     // current size of trust region (component-wise)
        double mCurrentMeritErrorCoeff;     // current penalty coefficient
    };
  }
}
