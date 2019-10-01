#include "private/muk.pch"
#include "bezier/BezierOptimizationModel.h"

#include "bezier/BezierCurvatureConstraint.h"
#include "bezier/BezierDistanceConstraint.h"
#include "bezier/BezierDistanceCost.h"
#include "bezier/BezierSplineCurvatureConstraint.h"
#include "bezier/BezierSplineCurvatureCost.h"
#include "bezier/BezierSplineDistanceConstraint.h"
#include "bezier/BezierSplineDistanceCost.h"
#include "bezier/BezierLengthCost.h"

#include "opt/GurobiSolver.h"
#include "opt/FunctionConstraint.h"

#include <boost/format.hpp>

namespace gris
{
namespace bezier
{
  /**
  */
  BezierOptimizationModel::BezierOptimizationModel()
    : mUseDistanceConstraint(true)
    , mUseDistanceCost(true)
    , mUseCurvatureConstraint(true)
    , mUseLengthCost(true)
    , mUseCurvatureCost(false)
  {
    mpProb = std::make_shared<opt::OptimizationProblem>();
    {
      auto pSolver = std::make_shared<opt::GurobiSolver>();
      mpProb->setSolver(pSolver);
    }
    mpOptimizer = std::make_shared<opt::BasicTrustRegion>();
    //mpOptimizer->setPenaltyIterationLimit(50);
    mpOptimizer->setSCOIterationLimit(50);
    mpOptimizer->setInitialTrustRegionSize(0.1);
    mpOptimizer->setTrustRegionScaleDownFactor(0.9);
    mpOptimizer->setTrustRegionScaleUpFactor(1.1);
    mpOptimizer->setMinTrustRegionSize(0.05);
    mpOptimizer->setProblem(mpProb);
    mpOptimizer->addCallback( [&] (auto* var1, auto& var2) { this->updateInternals(var1, var2); });
  }

  /** \brief This results in the reinitialization of the model's internal state

    The optimization model will have a number of states depending on the initial configuration.
  */
  void BezierOptimizationModel::setInitialStates(const std::vector<Vec3d>& points)
  {
    if (points.size() < 5)
    {
      throw std::exception("nothing to optimize, less than 5 states available");
    }
    mInitialTrajectory = points;
    mNumStates  = points.size();
    mNumConfigs = mNumStates - 2; // by design
    // create a configuration for each state that gets updated at each callback / iteration of mpOpt
    for (size_t i=0; i<mNumConfigs; ++i)
      mConfigs.push_back(std::make_shared<BezierConfig>(mKappaMax, mMinLength));
    // create variables for the convex optimization solver
    buildVariables();
    // the solver needs an initial value as a vector of doubles
    std::vector<double> initValues;
    for (size_t i(0); i<mNumStates; ++i)
    {
      const auto& p = points[i];
      initValues.push_back(p.x());
      initValues.push_back(p.y());
      initValues.push_back(p.z());
    }
    mpOptimizer->initialize(initValues);
  }

  /** \brief Creates variables (opt::Variable) for the waypoints
  */
  void BezierOptimizationModel::buildVariables()
  {
    std::vector<std::string> names;
    std::vector<double> lowerBounds;
    std::vector<double> upperBounds;
    const auto positionLowerBound    = -std::numeric_limits<double>::infinity();
    const auto positionUpperBound    =  std::numeric_limits<double>::infinity();
    // create the names for the variables
    for (size_t i=0; i < mNumStates; ++i)
    {
      names.push_back((boost::format("x_%02i_px") % i).str());
      names.push_back((boost::format("x_%02i_py") % i).str());
      names.push_back((boost::format("x_%02i_pz") % i).str());
      lowerBounds.push_back(positionLowerBound);
      lowerBounds.push_back(positionLowerBound);
      lowerBounds.push_back(positionLowerBound);
      upperBounds.push_back(positionUpperBound);
      upperBounds.push_back(positionUpperBound);
      upperBounds.push_back(positionUpperBound);
    }
    // create the variables
    const auto vars  = mpProb->createVariables(names, lowerBounds, upperBounds);
    const auto nCols = mDofsPosition;
    mPositionVariables.resize(mNumStates, nCols);
    // create a copy of the variables in the member matrix
    for (size_t i = 0; i < mNumStates; ++i)
      for (size_t j = 0; j < nCols; ++j)
        mPositionVariables(i, j) = vars[i*nCols + j];
  }

  /**
  */
  void BezierOptimizationModel::updateInternals(opt::OptimizationProblem*, std::vector<double>& x)
  {
    mCurrentEstimate = &x;
    // update the configs
    for (size_t i=0; i < mConfigs.size(); ++i) 
    {
      const auto& w1  = getWaypoint(i, x);
      const auto& w2  = getWaypoint(i+1, x);
      const auto& w3  = getWaypoint(i+2, x);
      mConfigs[i]->setStartPoint ( 0.5*(w1+w2));
      mConfigs[i]->setMiddlePoint( w2 );
      mConfigs[i]->setEndPoint   ( 0.5*(w2+w3));
      mConfigs[i]->computeSpline();
    }
  }
  
  /** \brief Extracts the variables for a Waypoint

    \param[in] idx index of idx'th row in the waypoint / position matrix
  */
  std::array<opt::Variable, 3> BezierOptimizationModel::getVariables(size_t idx) const
  {
    std::array<opt::Variable, 3> result;
    for(int i(0); i<3; ++i)
      result[i] = mPositionVariables(idx,i);
    return result;
  }

  /**
  */
  Vec3d BezierOptimizationModel::getWaypoint(size_t idx, const std::vector<double>& estimate) const
  {
    const auto vars     = getVariables(idx);
    const auto& x       = estimate;
    const auto waypoint = Vec3d(vars[0].value(x), vars[1].value(x), vars[2].value(x));
    return waypoint;
  }

  /**
  */
  std::vector<Vec3d> BezierOptimizationModel::getWaypoints(const std::vector<double>& estimate) const
  {
    std::vector<Vec3d> result;
    for (size_t i(0); i<mPositionVariables.rows(); ++i)
    {
      const auto waypoint = getWaypoint(i, estimate);
      result.push_back(waypoint);
    }
    return result;
  }

  /** \brief Creates cost functions and adds them to the optimizer
  */
  void BezierOptimizationModel::addCosts()
  {
    std::vector<ConstBezierConfigPtr> configs;
    for (auto& ptr : mConfigs)
      configs.push_back( std::dynamic_pointer_cast<const BezierConfig>(ptr) );
    if (mUseLengthCost)
      mpProb->addCost(std::make_shared<BezierLengthCost>(*this, mPositionVariables, configs, mLengthWeight ));
    if (mUseDistanceCost)
    {
      /*auto pDistCost = std::make_shared<BezierDistanceCost>(*this, mpColl.get());
      pDistCost->setDistanceThreshold(mBezierDistanceCostMargin);
      pDistCost->setWeigth(mObstacleWeight);
      mpProb->addCost(pDistCost);*/
      for (size_t i=0; i < mConfigs.size(); ++i)
      {
        // distance
        auto pDistCost = std::make_shared<BezierSplineDistanceCost>(*this, mpColl.get());
        pDistCost->setDistanceThreshold(mBezierDistanceCostMargin);
        pDistCost->setWeigth(mObstacleWeight);
        pDistCost->setBezierSplineIndex(i);
        mpProb->addCost(pDistCost);
      }
    }
    if (mUseCurvatureCost)
    {
      for (size_t i=0; i < mConfigs.size(); ++i)
      {
        auto pCost = std::make_shared<BezierSplineCurvatureCost>(*this);
        pCost->setWeigth(mCurvatureWeight);
        pCost->setBezierSplineIndex(i);
        pCost->setMaxIndex(mConfigs.size()-1);
        mpProb->addCost(pCost);
      }
    }
  }

  /**
  */
  void BezierOptimizationModel::addConstraints()
  {
    // 2 fixed start + 2 fixed goal waypoints are used to ensure the direction at the start and goal.
    const auto N = mInitialTrajectory.size();
    for (int j(0); j<3; ++j)
    {
      mpProb->addLinearConstraint( opt::AffineExpression(mPositionVariables(0, j))   - mInitialTrajectory[0][j]  , opt::enEQ);
      mpProb->addLinearConstraint( opt::AffineExpression(mPositionVariables(1, j))   - mInitialTrajectory[1][j]  , opt::enEQ);
      mpProb->addLinearConstraint( opt::AffineExpression(mPositionVariables(N-2, j)) - mInitialTrajectory[N-2][j], opt::enEQ);
      mpProb->addLinearConstraint( opt::AffineExpression(mPositionVariables(N-1, j)) - mInitialTrajectory[N-1][j], opt::enEQ);
    }
    // individual Curvature constraints
    std::cout << "create " << mConfigs.size() << " constraints" << std::endl;
    for (size_t i=0; i < mConfigs.size(); ++i)
    {
      // curvature
      if (mUseCurvatureConstraint)
      {
        auto pNextConstraint = std::make_shared<BezierCurvatureConstraint>(*this, i);
        pNextConstraint->setMaxIndex(mConfigs.size() - 1);
        const auto vars   = pNextConstraint->getVariables();
        const auto coeffs = Eigen::VectorXd::Ones(vars.size());
        auto cnt = std::make_shared<opt::FunctionConstraint>(pNextConstraint, vars, coeffs, opt::enEQ, (boost::format("kappa_%i") % i).str());
        mpProb->addConstraint(cnt);
        /*auto pCurvCnt = std::make_shared<BezierSplineCurvatureConstraint>(*this, i);
        mpProb->addConstraint(pCurvCnt);*/
      }
      // distance
      if (mUseDistanceConstraint)
      {
        /*auto pDistConstraint = std::make_shared<BezierDistanceConstraint>(*this, mpColl.get());
        pDistConstraint->setDistanceThreshold(mBezierDistanceConstraintMargin);
        mpProb->addConstraint(pDistConstraint);*/
        // distance
        auto pDistCost = std::make_shared<BezierSplineDistanceConstraint>(*this, mpColl.get());
        //LOG_LINE << "adding distance constraint with " << mBezierDistanceConstraintMargin;
        pDistCost->setDistanceThreshold(mBezierDistanceConstraintMargin);
        pDistCost->setBezierSplineIndex(i);
        mpProb->addConstraint(pDistCost);
      }
    }
  }
}
}