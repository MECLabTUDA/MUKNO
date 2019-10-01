#pragma once

#include "bezier/bezier_types.h"
#include "bezier/BezierConfig.h"

#include "opt/BasicTrustRegion.h"
#include "opt/OptimizationProblem.h"
#include "opt/Optimizer.h"

#include "opt_types.h"

#include "MukCommon/ICollisionDetector.h"

#include <memory>

namespace gris
{
  namespace bezier
  {
    /** \brief Models the Optimization Problem by providing a definition of the states and proper access to it.

      Transforms an initial trajectory to the implementation of an optimization problem, including 
        - the variable representation (the 3D positions -> #mPositionVariables), 
        - the Bezier Spline representations
        - a collision detector (#mpColl) 
        - more stuff like the optimizer (#mpOptimizer), etc...
    */
    class BezierOptimizationModel
    {
      public:
        BezierOptimizationModel();

      public:
        void setInitialStates(const std::vector<Vec3d>& points);
        void addCosts();
        void addConstraints();
        void setCollisionDetector(std::shared_ptr<muk::ICollisionDetector> pColl) { mpColl = pColl; }

      public:
        opt::OptimizationProblem&   getProblem()          const { return *mpProb; }
        opt::Optimizer&             getOptimizer()              { return *mpOptimizer; }
        const muk::VariableMatrix&  getStateVariables()   const { return mPositionVariables; }
        const std::vector<BezierConfigPtr>& getConfigs()  const { return mConfigs;  }

        void                          setKappaMax(double d)               { mKappaMax = d; }
        double                        getKappaMax()                 const { return mKappaMax; }
        void                          setMinLength(double d)              { mMinLength = d; }
        double                        getMinLength()                const { return mMinLength; }
        void                          setDistanceConstraintMargin(double d)    { mBezierDistanceConstraintMargin = d; }
        double                        getDistanceConstraintMargin() const      { return mBezierDistanceConstraintMargin; }
        void                          setDistanceCostMargin(double d)     { mBezierDistanceCostMargin = d; }
        double                        getDistanceCostMargin()       const { return mBezierDistanceCostMargin; }
        void                          setDistanceCost(bool b)             { mUseDistanceCost = b;  }
        void                          setLengthCost(bool b)               { mUseLengthCost = b; }
        void                          setCurvatureCost(bool b)            { mUseCurvatureCost = b; }
        void                          setDistanceWeight(double val)       { mObstacleWeight = val;  }
        void                          setLengthWeight(double val)         { mLengthWeight   = val; }
        void                          setCurvatureWeight(double val)      { mCurvatureWeight = val; }
        void                          setDistanceConstraint(bool b)       { mUseDistanceConstraint = b;  }
        void                          setCurvatureConstraint(bool b)      { mUseCurvatureConstraint = b; }
        std::array<opt::Variable, 3>  getVariables(size_t idx)      const;
        Vec3d                         getWaypoint(size_t idx, const std::vector<double>& estimate)  const;
        std::vector<Vec3d>            getWaypoints(const std::vector<double>& estimate)             const;

      private:
        void buildVariables();
        void updateInternals(opt::OptimizationProblem*, std::vector<double>& x);

      private:
        size_t mNumStates;
        size_t mNumConfigs;
        static const size_t mDofsPosition = 3;

        double mKappaMax  = 1.0;
        double mMinLength = 0.25;
        double mBezierDistanceCostMargin = 1.0;
        double mBezierDistanceConstraintMargin = 1.0;

        bool mUseDistanceConstraint;
        bool mUseDistanceCost;
        bool mUseCurvatureConstraint;
        bool mUseCurvatureCost;
        bool mUseLengthCost;

        double mLengthWeight   = 1.0;
        double mObstacleWeight = 1.0;
        double mCurvatureWeight = 1.0;

        std::vector<Vec3d> mInitialTrajectory;
        muk::VariableMatrix mPositionVariables; /// just a copied block of mStateVariableMatrix for better code understanding
        std::shared_ptr<opt::OptimizationProblem> mpProb;
        std::shared_ptr<opt::BasicTrustRegion>    mpOptimizer;
        std::shared_ptr<muk::ICollisionDetector>  mpColl;
        std::vector<BezierConfigPtr> mConfigs;
        std::vector<double>*  mCurrentEstimate;
    };
  }
}
