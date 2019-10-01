#include "private/muk.pch"
#include "opt/BasicTrustRegion.h"

#include "opt/opt_util.h"

#include <boost/format.hpp>

#include <assert.h>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <sstream>

namespace
{
  /**
  */
  std::string toString(const std::vector<double>& v)
  {
    std::stringstream ss;
    for (const auto d : v)
      ss << d << " ";
    return ss.str();
  }

  /** \brief Prints a formatted output describing the individual cost and constraint values
  */
  void printCostInfo(const std::vector<double>& old_cost_vals, const std::vector<double>& model_cost_vals, const std::vector<double>& new_cost_vals,
    const std::vector<double>& old_cnt_vals,  const std::vector<double>& model_cnt_vals,  const std::vector<double>& new_cnt_vals,
    const std::vector<std::string>& cost_names, const std::vector<std::string>& cnt_names, double merit_coeff) ;
}

namespace gris
{
namespace opt
{
  /**
  */
  BasicTrustRegion::BasicTrustRegion() 
  {
    initParameters();
  }

  /**
  */
  BasicTrustRegion::BasicTrustRegion(OptimizationProblemPtr prob)
  {
    initParameters();
    setProblem(prob);
    setVerbose(false);
  }

  /**
  */
  void BasicTrustRegion::initParameters() 
  {
    mConstraintTolerance        = 1e-4;
    mInitialMeritErrorCoeff     = 10;
    mInitialTrustRegionSize     = 0.1;

    mIterationLimitPenalty      = 15;
    mMeritIncreaseRatio         = 10;
    mMeritImproveRatioThreshold = 0.25;

    mIterationLimitSCO          = 50;

    mMinTrustRegionSize         = 1e-4;
    mMaxTrustRegionSize         = 1.0;
    mTrustRegionScaleDownFactor = 0.5;
    mTrustRegionScaleUpFactor   = 1.5;
    mTrustRegionIterationLimit  = 10;

    mModelImprovementThreshold  = 1e-4;
    mModelImprovementRatioThreshold = -std::numeric_limits<double>::infinity();
  }

  /**
  */
  void BasicTrustRegion::setVerbose(bool val) 
  {
    mVerbose = val; 
    mpSolver->setVerbose(val);
  }

  /**
  */
  void BasicTrustRegion::setProblem(OptimizationProblemPtr prob)
  {
    Optimizer::setProblem(prob);
    mpSolver = prob->getModel();
  }

  /** \brief restricts the bounds to a tube around the current estimate of size = #mCurrentTrustRegionSize.

    \param[in] x current estimate
  */
  void BasicTrustRegion::restrictBoundsToTrustRegion(const std::vector<double>& x)
  {
    auto& vars = mpOptProb->getVars();
    assert(vars.size() == x.size());
    auto& lb = mpOptProb->getLowerBounds();
    auto& ub = mpOptProb->getUpperBounds();
    
    std::vector<double> lbtrust(x.size());
    std::vector<double> ubtrust(x.size());
    for (size_t i(0); i<x.size(); ++i)
    {
      lbtrust[i] = std::max(x[i] - mCurrentTrustRegionSize, lb[i]);
      ubtrust[i] = std::min(x[i] + mCurrentTrustRegionSize, ub[i]);
    }
    mpSolver->setVarBounds(vars, lbtrust, ubtrust);
  }

  /** \brief The start of the optimization procedure.

    Lots of setup
    then call to "penaltyRuns(estimate, w)", which does the optimization
    rest is postprocessing
  */
  EnOptimizerResult BasicTrustRegion::optimize()
  {
    std::vector<double>& estimate = mResults.x; // just so I don't have to rewrite code
    if (estimate.empty())
      throw std::exception("No initial estimate / solution available!");
    if ( ! mpOptProb)
      throw std::exception("No optimization problem available");
    mCurrentTrustRegionSize = mInitialTrustRegionSize;
    mCurrentMeritErrorCoeff = mInitialMeritErrorCoeff;
    Worker w;
    w.constraints   = mpOptProb->getConstraints();
    w.loopState     = enInitialized;
    w.iterPenalty   = 0;
    w.iterConvexify = 0;
    
    estimate = mpOptProb->getClosestFeasiblePoint(estimate);
    assert(estimate.size() == mpOptProb->getVars().size());
    assert(mpOptProb->getCosts().size() > 0 || constraints.size() > 0);
    penaltyRuns(estimate, w);

    // translate internal state to result state
    switch (w.loopState)
    {
      case enConstraintsSatisfied:
        mResults.status = enOptimizerConverged; break;
      case enInitialized:
        mResults.status = enBadInitialization; break;
      case enTrustRegionFailed:
        mResults.status = enOptimizerFailed; break;
      case enSQPFailed:
        mResults.status = enSCOIterationLimitReached; break;
      case enPenaltyRunFailed:
        mResults.status = enPenaltyIterationLimitReached; break;
    }
    mResults.total_cost = std::accumulate(mResults.cost_vals.begin(), mResults.cost_vals.end(), 0.0);
    callCallbacks(estimate); // why again?
    // determine and print out violated constraints
    std::vector<size_t> ids;
    for (size_t i(0); i<mResults.cnt_viols.size(); ++i)
    {
      if (mResults.cnt_viols[i] >= mConstraintTolerance)
        ids.push_back(i);
    }
    if ( ! ids.empty())
    {
      LOG << "violated constraints:\n";
      for (size_t i(0); i<ids.size(); ++i)
      {
        const auto& val  = mResults.cnt_viols[ids[i]];
        const auto& name = mpOptProb->getConstraints()[ids[i]]->getName();
        LOG << "   " << name << ": " << val << "\n";
      }
      LOG_LINE;
    }
    else
    {
      mResults.improved = true;
    }
    LOG_LINE << "\n==================\n" << mResults << "\n==================";

    return w.globalStatus;
  }

  /** \brief Perform an iterative sequential convex optimization step based on a merit function

    possible cases
      - one of the inner loops failed  -> stop (failed)
      - all constraints were satisfied -> stop (success)
      - some constraints are still unsatisfied -> increase penalties and do the next SCO run
  */
  void BasicTrustRegion::penaltyRuns(std::vector<double>& estimate, Worker& w)
  {
    while (w.iterPenalty < mIterationLimitPenalty)
    {
      // run an attempt solve the subprolbem
      sqpRuns(estimate, w);
      // one of the inner loops failed or penalty is too small for further improvement
      if ( w.loopState == enTrustRegionFailed
        || w.loopState == enSQPFailed)
        return;
      // check state after successfull inner loops
      const auto vecMax = std::max_element(mResults.cnt_viols.begin(), mResults.cnt_viols.end());
      if (mResults.cnt_viols.empty() || *vecMax < mConstraintTolerance) 
      {
        if (mResults.cnt_viols.size() > 0)
        {
          if (mVerbose)
            LOG_INFO("All constraints are satisfied (to tolerance %.2e)!", mConstraintTolerance);
        }
        w.loopState = enConstraintsSatisfied;
        return;
      }
      else
      {
        if (mVerbose)
          LOG_INFO("not all constraints are satisfied. increasing penalties");
        mCurrentMeritErrorCoeff *= mMeritIncreaseRatio;
        mCurrentTrustRegionSize = std::max(mCurrentTrustRegionSize, mMinTrustRegionSize / mTrustRegionScaleDownFactor * 1.5);
        w.loopState = enConstraintsViolated;
      }
      ++w.iterPenalty;
    }
    w.loopState = enPenaltyRunFailed;
    if (mVerbose)
      LOG_INFO("optimization couldn't satisfy all constraints");
  }

  /** \brief convexifies costs and constraints and then solves iteratively in a trust region

    This loop does nothing but convexifying. Solving is done within the trust region (inner loop).

    4 possible cases
    - inner loop (trust region) failed -> stop everything
    - trust region became too small    -> adjust penalty (???? this is apparently the normal case)
    - iteration limit reached -> stop everything 
  */
  void BasicTrustRegion::sqpRuns(std::vector<double>& estimate, Worker& w)
  {
    // reset sqp loop iteration
    for (w.iterConvexify = 1; w.iterConvexify < mIterationLimitSCO; ++w.iterConvexify) /* sqp loop */
    {
      callCallbacks(estimate); // plot or adjust some values based on the estimate (e.g. forward integration)
      
      if (mResults.cost_vals.empty() && mResults.cnt_viols.empty()) // only happens on the first iteration
      { 
        for (const auto& cnt : w.constraints)
          mResults.cnt_viols.push_back( cnt->violation(estimate) );
        for (const auto& cost : mpOptProb->getCosts())
          mResults.cost_vals.push_back( cost->value(estimate) );
        ++mResults.n_func_evals;
      }

      w.convexifiedCosts       = convexifyCosts(mpOptProb->getCosts(), estimate);
      w.convexifiedConstraints = convexifyConstraints(w.constraints, estimate);
      auto modelConstraints    = convertConstraintToCosts(w.convexifiedConstraints);
      mpSolver->update();

      for(const auto& cost : w.convexifiedCosts)
        cost->addConstraintsToModel();
      for(const auto& cost : modelConstraints)
        cost->addConstraintsToModel();
      mpSolver->update();

      QuadraticExpression objective;
      for(const auto& co : w.convexifiedCosts)
        objective += co->mQuad;
      for(const auto& co : modelConstraints)
        objective += co->mQuad;
      //objective.make_stable();
      mpSolver->setObjective(objective);

      trustBoxRuns(estimate, w);

      if ( w.loopState == enTrustRegionFailed
        || w.loopState == enCheckIfConverged)
        return;
    }
    if (mVerbose)
      LOG_INFO("sqp iteration limit reached");
    w.loopState = enSQPFailed;
  }

  /** \brief Optimizes the problem in a local trust region. Repeats on decreasing trust regions until an improvement occurs.

    4 possible cases (1 where the loop continues, 3 were it breaks)
     - exact merit worsened      -> shrink region and continue solving in a smaller region
     - problem became infeasible -> stop
     - approximated merit improved, but diff is very small -> stop
     - exact merit improved                                -> stop
  */
  void BasicTrustRegion::trustBoxRuns(std::vector<double>& estimate, Worker& w)
  {
    w.iterTrustBox = 0;
    while (mCurrentTrustRegionSize >= mMinTrustRegionSize)
    {
      restrictBoundsToTrustRegion(estimate);
      CvxOptStatus status = mpSolver->optimize();
      ++mResults.n_qp_solves;

      if (status != enConvexSubProbSolved)
      {
        const auto dir = std::string("../resources/");
        const auto fn1 = std::string("fail.lp");
        const auto fn2 = std::string("fail.ilp");
        LOG_LINE << "Convex solver failed! Saving model & IIS, if available, under " << dir << " as " << fn1 << " & " << fn2;
        mpSolver->writeToFile(dir+fn1);
        if (status != enConvexSubProbFailed)
          mpSolver->writeToFile(dir+fn2);
        w.loopState = enTrustRegionFailed;
        return;
      }

      const auto vars = mpSolver->getVars();
      auto model_var_vals = mpSolver->getVarValues(vars);
      std::vector<double> model_cost_vals; // these are convexified costs values
      for (const auto& cost : w.convexifiedCosts)
        model_cost_vals.push_back(cost->value(model_var_vals));
      std::vector<double> model_cnt_viols; // these are convexified constraint values
      for (const auto& cost : w.convexifiedConstraints)
        model_cnt_viols.push_back(cost->violation(model_var_vals));

      // the n variables of the OptProb happen to be the first n variables in the Model
      std::vector<double> new_x (model_var_vals.begin(), model_var_vals.begin() + estimate.size());
      //callCallbacks(new_x); // plot or adjust some values based on the estimate (e.g. forward integration)

      auto new_cost_vals = evaluateCosts(mpOptProb->getCosts(), new_x); // these are the real cost values (potentially based on nonconvex functions)
      auto new_cnt_viols = evaluateCntViols(w.constraints, new_x);      // these are the real constraint values
      ++mResults.n_func_evals;

      auto l_sum = [&] (const std::vector<double>& v)
      {
        return std::accumulate(v.begin(), v.end(), 0.0);
      };
      double old_merit   = l_sum(mResults.cost_vals) +  mCurrentMeritErrorCoeff  *  l_sum(mResults.cnt_viols);
      double model_merit = l_sum(model_cost_vals)    +  mCurrentMeritErrorCoeff  *  l_sum(model_cnt_viols);
      double new_merit   = l_sum(new_cost_vals)      +  mCurrentMeritErrorCoeff  *  l_sum(new_cnt_viols);

      double approx_merit_improve = old_merit - model_merit;
      double exact_merit_improve  = old_merit - new_merit;
      double merit_improve_ratio  = exact_merit_improve / approx_merit_improve;

      if (mVerbose)
      {
        LOG_LINE << " ";
        LOG_LINE << (boost::format("penalty iteration  %2i, sqp iteration %2i, trust box iteration %2i") % (w.iterPenalty+1) % (w.iterConvexify+1) % (w.iterTrustBox+1)).str();
        printCostInfo(mResults.cost_vals, model_cost_vals, new_cost_vals,
          mResults.cnt_viols, model_cnt_viols, new_cnt_viols, mpOptProb->getCostNames(),
          mpOptProb->getConstraintNames(), mCurrentMeritErrorCoeff);
        LOG_LINE << (boost::format("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n") % "TOTAL" % old_merit % approx_merit_improve % exact_merit_improve % merit_improve_ratio).str();
      }

      if (approx_merit_improve < -1e-5)  /// thats just a hint
      {
        LOG_LINE << (boost::format("approximate merit function got worse (%.3e). (convexification is probably wrong to zeroth order)\n") % approx_merit_improve).str();
      }

      const auto meritIsSmall      = approx_merit_improve < mModelImprovementThreshold;
      const auto meritRatioIsSmall = approx_merit_improve / old_merit < mModelImprovementRatioThreshold;
      if (meritIsSmall || meritRatioIsSmall)
      {
        if (meritIsSmall)
        {
          LOG_LINE << (boost::format("converged because improvement was small (%.3e < %.3e)") % approx_merit_improve % mModelImprovementThreshold).str();
        }
        else
        {
          LOG_LINE << (boost::format("converged because improvement ratio was small (%.3e < %.3e)") % (approx_merit_improve/old_merit) % mModelImprovementRatioThreshold).str();
        }
        w.loopState = enCheckIfConverged;
        return;
      }
      else
      {
        if (exact_merit_improve < 0 || merit_improve_ratio < mMeritImproveRatioThreshold)
        {
          // this is the only condition, where the loop migth continue
          mCurrentTrustRegionSize *= mTrustRegionScaleDownFactor;
          if (mVerbose)
          {
            const auto reason = exact_merit_improve < 0 ? "exact merit improve < 0" : "improve ratio too low";
            LOG_LINE << (boost::format("%s. shrunk trust region. new box size: %.4f") % reason % mCurrentTrustRegionSize).str();
          }
        }
        else
        {
          estimate = new_x; // why accept only here?
          mResults.cost_vals = new_cost_vals;
          mResults.cnt_viols = new_cnt_viols;
          mCurrentTrustRegionSize *= mTrustRegionScaleUpFactor;
          mCurrentTrustRegionSize  = std::min(mCurrentTrustRegionSize, mMaxTrustRegionSize);
          if (mVerbose)
            LOG_LINE << (boost::format("expanded trust region. new box size: %.4f") % mCurrentTrustRegionSize).str();
          return;
        }
      }
      ++w.iterTrustBox;
    }
    if (mVerbose)
      LOG_LINE << "converged because trust region is tiny";
    w.loopState = enCheckIfConverged;
  }

  /**
  */
  std::vector<double> BasicTrustRegion::evaluateCosts(const std::vector<CostPtr>& costs, const std::vector<double>& x) const
  {
    std::vector<double> out(costs.size());
    for (size_t i=0; i < costs.size(); ++i)
      out[i] = costs[i]->value(x);
    return out;
  }

  /**
  */
  std::vector<double> BasicTrustRegion::evaluateCntViols(const std::vector<ConstraintPtr>& cnts, const std::vector<double>& x) const
  {
    std::vector<double> out(cnts.size());
    for (size_t i = 0; i < cnts.size(); ++i)
      out[i] = cnts[i]->violation(x);
    return out;
  }

  /**
  */
  std::vector<double> BasicTrustRegion::evaluateModelCosts(const std::vector<ConvexObjectivePtr>& costs, const std::vector<double>& x) const
  {
    std::vector<double> out(costs.size());
    for (size_t i=0; i < costs.size(); ++i)
      out[i] = costs[i]->value(x);
    return out;
  }

  /**
  */
  std::vector<double> BasicTrustRegion::evaluateModelCntViols(const std::vector<ConvexConstraintsPtr>& cnts, const std::vector<double>& x) const
  {
    std::vector<double> out(cnts.size());
    for (size_t i=0; i < cnts.size(); ++i)
      out[i] = cnts[i]->violation(x);
    return out;
  }

  /**
  */
  std::vector<ConvexObjectivePtr> BasicTrustRegion::convexifyCosts(const std::vector<CostPtr>& costs, const std::vector<double>& estimate) const
  {
    std::vector<ConvexObjectivePtr> result;
    std::transform(costs.begin(), costs.end(), std::back_inserter(result), 
      [&] (const auto& cost)
    {
      return cost->convexify(estimate, *mpSolver);
    });
    return result;
  }

  /**
  */
  std::vector<ConvexConstraintsPtr> BasicTrustRegion::convexifyConstraints(const std::vector<ConstraintPtr>& constraints, const std::vector<double>& estimate) const
  {
    std::vector<ConvexConstraintsPtr> result;
    std::transform(constraints.begin(), constraints.end(), std::back_inserter(result), 
      [&] (const auto& constraint)
      {
        return constraint->convexify(estimate, *mpSolver);
      });
    return result;
  }

  /**
  */
  std::vector<ConvexObjectivePtr> BasicTrustRegion::convertConstraintToCosts(const std::vector<ConvexConstraintsPtr>& constraints) const
  {
    std::vector<ConvexObjectivePtr> result;
    for(const auto& cnt : constraints)
    {
      auto obj = std::make_shared<ConvexObjective>(mpSolver.get());
      for (const auto& aff : cnt->mEqualities)
      {
        obj->addAbs(aff, mCurrentMeritErrorCoeff);
      }
      for (const auto& aff : cnt->mInequalities)
      {
        obj->addHinge(aff, mCurrentMeritErrorCoeff);
      }
      result.push_back(obj);
    }
    return result;
  }
}
}

namespace
{
  /**
  */
  void printCostInfo(const std::vector<double>& old_cost_vals
    , const std::vector<double>& model_cost_vals
    , const std::vector<double>& new_cost_vals
    , const std::vector<double>& old_cnt_vals
    , const std::vector<double>& model_cnt_vals
    , const std::vector<double>& new_cnt_vals
    , const std::vector<std::string>& cost_names
    , const std::vector<std::string>& cnt_names
    , double merit_coeff) 
  {
    printf("%15s | %10s | %10s | %10s | %10s\n", "", "old merit", "tmp merit", "new merit", "ratio");
    printf("%15s | %10s---%10s---%10s---%10s\n", "COSTS", "----------", "----------", "----------", "----------");
    for (size_t i=0; i < old_cost_vals.size(); ++i)
    {
      double approx_improve = old_cost_vals[i] - model_cost_vals[i];
      double exact_improve  = old_cost_vals[i] - new_cost_vals[i];
      if (fabs(approx_improve) > 1e-8) 
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n", cost_names[i].c_str(), old_cost_vals[i], approx_improve, exact_improve, exact_improve/approx_improve);
      else
        printf("%15s | %10.3e | %10.3e | %10.3e | %10s\n",   cost_names[i].c_str(), old_cost_vals[i], approx_improve, exact_improve, "  ------  ");
    }
    if (cnt_names.size() == 0) return;
    printf("%15s | %10s---%10s---%10s---%10s\n", "CONSTRAINTS", "----------", "----------", "----------", "----------");
    for (size_t i=0; i < old_cnt_vals.size(); ++i)
    {
      double approx_improve = old_cnt_vals[i] - model_cnt_vals[i];
      double exact_improve = old_cnt_vals[i] - new_cnt_vals[i];
      if (fabs(approx_improve) > 1e-8)
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n", cnt_names[i].c_str(), merit_coeff*old_cnt_vals[i], merit_coeff*approx_improve, merit_coeff*exact_improve, exact_improve/approx_improve); 
      else 
        printf("%15s | %10.3e | %10.3e | %10.3e | %10s\n", cnt_names[i].c_str(), merit_coeff*old_cnt_vals[i], merit_coeff*approx_improve, merit_coeff*exact_improve, "  ------  "); 
    }
  }
}