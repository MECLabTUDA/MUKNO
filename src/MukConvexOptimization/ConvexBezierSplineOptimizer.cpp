#include "private/muk.pch"
#include "ConvexBezierSplineOptimizer.h"

#include "bezier/BezierOptimizationModel.h"

#include "opt/BasicTrustRegion.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/muk_dynamic_property_tools.h"
#include "MukCommon/MukProblemDefinition.h"
#include "MukCommon/OptimizerFactory.h"

namespace gris
{
namespace muk
{
  REGISTER_OPTIMIZER(ConvexBezierSplineOptimizer);

  // ----------------------------------------------------------------------
  /** Ctor
  */
  ConvexBezierSplineOptimizer::ConvexBezierSplineOptimizer()
    : mMaxDistancePenalty(5.0)
    , mUseLengthCost(false)
    , mUseDistanceCost(true)
    , mUseCurvatureConstraint(true)
    , mUseCurvatureCost(false)
    , mUseDistanceConstraint(false)
    , mInitialTrustRegionSize(0.1)
    , mVerbose(false)
    , mLengthWeight(1.0)
    , mObstacleWeight(1.0)
    , mCurvatureWeight(1.0)
    , mMaxTrustRegionSize(1.0)
    , mMinTrustRegionSize(0.1)
  {
    declareProperty<double>("InitialTrustRegionSize", MUK_D_SET(double, mInitialTrustRegionSize), MUK_D_GET(mInitialTrustRegionSize));
    declareProperty<double>("MinTrustRegionSize", MUK_D_SET(double, mMinTrustRegionSize), MUK_D_GET(mMinTrustRegionSize));
    declareProperty<double>("MaxTrustRegionSize", MUK_D_SET(double, mMaxTrustRegionSize), MUK_D_GET(mMaxTrustRegionSize));

    declareProperty<double>("MaxDistancePenalty", MUK_SET(double, setMaxDistancePenalty), MUK_GET(getMaxDistancePenalty));

    declareProperty<bool>("DistanceConstraint",   MUK_D_SET(bool, mUseDistanceConstraint), MUK_D_GET(mUseDistanceConstraint));
    declareProperty<bool>("DistanceCost",         MUK_D_SET(bool, mUseDistanceCost), MUK_D_GET(mUseDistanceCost));
    declareProperty<bool>("CurvatureConstraint",  MUK_D_SET(bool, mUseCurvatureConstraint), MUK_D_GET(mUseCurvatureConstraint));
    declareProperty<bool>("CurvatureCost",        MUK_D_SET(bool, mUseCurvatureCost), MUK_D_GET(mUseCurvatureCost));
    declareProperty<bool>("LengthCost",           MUK_D_SET(bool, mUseLengthCost), MUK_D_GET(mUseLengthCost));
    declareProperty<bool>("Verbose",              MUK_D_SET(bool, mVerbose), MUK_D_GET(mVerbose));

    declareProperty<double>("WeightLength",       MUK_D_SET(double, mLengthWeight), MUK_D_GET(mLengthWeight));
    declareProperty<double>("WeightObstacles",    MUK_D_SET(double, mObstacleWeight), MUK_D_GET(mObstacleWeight));
    declareProperty<double>("WeightCurvature",    MUK_D_SET(double, mCurvatureWeight), MUK_D_GET(mCurvatureWeight));
  }

  // ----------------------------------------------------------------------
  /**
  */
  MukPath ConvexBezierSplineOptimizer::calculate(const MukPath& input) const
  {
    auto probDef = mpProbDef.lock();
    const auto r = probDef->getRadius();
    const auto d = probDef->getSafetyDist();
    using namespace opt;
    using namespace bezier;
    BezierOptimizationModel model;
    model.setCollisionDetector(mpCollision);
    model.setKappaMax( mKappa );
    model.setMinLength( mMaxStepSize );
    std::vector<Vec3d> initPoints;
    
    std::transform(input.getStates().begin(), input.getStates().end(), std::back_inserter(initPoints), [&] (const auto& state) { return state.coords; });

    model.setInitialStates(initPoints);
    // params
    model.setDistanceConstraintMargin(r+d);
    model.setDistanceCostMargin(mMaxDistancePenalty);
    // activate cost & constraints
    model.setCurvatureConstraint(mUseCurvatureConstraint);
    model.setCurvatureCost(mUseCurvatureCost);
    model.setDistanceConstraint(mUseDistanceConstraint);
    model.setDistanceCost(mUseDistanceCost);
    model.setLengthCost(mUseLengthCost);
    // set weights
    model.setCurvatureWeight(mCurvatureWeight);
    model.setDistanceWeight(mObstacleWeight);
    model.setLengthWeight(mLengthWeight);
    // set model params
    auto& opt = dynamic_cast<BasicTrustRegion&>(model.getOptimizer());
    opt.setInitialTrustRegionSize(mInitialTrustRegionSize);
    opt.setMaxTrustRegionSize(mMaxTrustRegionSize);
    opt.setMinTrustRegionSize(mMinTrustRegionSize);
    opt.setVerbose(mVerbose);
    // build
    model.addCosts();
    model.addConstraints();
    model.getOptimizer().optimize();

    mValid = model.getOptimizer().results().improved;
    if ( ! mValid)
    {
      LOG_LINE << "Optimization failed (" << model.getOptimizer().results().status << ")";
      return input;
    }
    LOG_LINE << "Optimization succeeded";
    const auto& estimate = model.getOptimizer().results().x;
    std::vector<MukState> states;
    
    for (size_t i(0); i<estimate.size() / 3; ++i)
    {
      const auto p = Vec3d(estimate[3*i], estimate[3*i+1], estimate[3*i+2]);
      states.push_back(MukState(p, Vec3d(0,0,0)));
    }

    if ( states.size() > 1 )
    {
      states[0].tangent = (states[1].coords - states[0].coords).normalized();
    }
    for (size_t i(1); i<states.size(); ++i)
    {
      states[i].tangent = (states[i].coords - states[i-1].coords).normalized();
    }
    MukPath p = input;
    p.setStates(states);
    return p;
  }
}
}