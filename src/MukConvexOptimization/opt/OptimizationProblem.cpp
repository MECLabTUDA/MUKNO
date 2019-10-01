#include "opt/OptimizationProblem.h"

#include <assert.h>
#include <algorithm>
#include <iterator>
#include <iostream>

namespace gris
{
namespace opt
{
  /**
  */
  OptimizationProblem::OptimizationProblem()
    : mpSolver(nullptr) 
  {
  }

  /** \brief Creates a variable with lower and upper bound
  */
  Variable OptimizationProblem::createVariable(const std::string& name, double lb, double ub)
  {
    mVariables.push_back(mpSolver->addVar(name, lb, ub));
    mLowerBounds.push_back(lb);
    mUpperBounds.push_back(ub);
    mpSolver->update();
    return mVariables.back();
  }

  /**
  */
  std::vector<Variable> OptimizationProblem::createVariables(const std::vector<std::string>& var_names) 
  {
    const auto limit = std::numeric_limits<double>::infinity();
    return createVariables(
      var_names, 
      std::vector<double>(var_names.size(), -limit), 
      std::vector<double>(var_names.size(), limit));
  }

  /** \brief convenience function to add multiple variables

    difference to adding step by step: updates the solver only at the end
  */
  std::vector<Variable> OptimizationProblem::createVariables(const std::vector<std::string>& var_names, const std::vector<double>& lb, const std::vector<double>& ub)
  {
    size_t n_add = var_names.size();
    assert(lb.size() == n_add);
    assert(ub.size() == n_add);
    for (size_t i(0); i < var_names.size(); ++i)
    {
      mVariables.push_back(mpSolver->addVar(var_names[i], lb[i], ub[i]));
      mLowerBounds.push_back(lb[i]);
      mUpperBounds.push_back(ub[i]);
    }
    mpSolver->update();
    return std::vector<Variable>(mVariables.end()-n_add, mVariables.end());
  }

  /**
  */
  void OptimizationProblem::setLowerBounds(const std::vector<double>& lb) 
  {
    assert(lb.size() == mVariables.size());
    mLowerBounds = lb;
  }

  /**
  */
  void OptimizationProblem::setUpperBounds(const std::vector<double>& ub) 
  {
    assert(ub.size() == mVariables.size());
    mUpperBounds = ub;
  }

  /**
  */
  void OptimizationProblem::setLowerBounds(const std::vector<double>& lb, const std::vector<Variable>& vars) 
  {
    for (size_t i(0); i<vars.size(); ++i) 
    {
      mLowerBounds[vars[i].getRep()->getIndex()] = lb[i];
    }
  }

  /**
  */
  void OptimizationProblem::setUpperBounds(const std::vector<double>& ub, const std::vector<Variable>& vars)
  {
    for (size_t i(0); i<vars.size(); ++i) 
    {
      mUpperBounds[vars[i].getRep()->getIndex()] = ub[i];
    }
  }

  /**
  */
  void OptimizationProblem::addCost(CostPtr cost)
  {
    mCosts.push_back(cost);
  }

  /**
  */
  void OptimizationProblem::addConstraint(ConstraintPtr cnt) 
  {
    if (cnt->type() == enEQ) 
      addEqConstraint(cnt);
    else 
      addIneqConstraint(cnt);
  }

  /**
  */
  void OptimizationProblem::addEqConstraint(ConstraintPtr cnt)
  {
    assert (cnt->type() == enEQ);
    mEqualities.push_back(cnt);
  }

  /**
  */
  void OptimizationProblem::addIneqConstraint(ConstraintPtr cnt) 
  {
    assert (cnt->type() == enINEQ);
    mInequalities.push_back(cnt);
  }

  /**
  */
  std::vector<ConstraintPtr> OptimizationProblem::getConstraints() const 
  {
    std::vector<ConstraintPtr> out;
    out.reserve(mEqualities.size() + mInequalities.size());
    out.insert(out.end(), mEqualities.begin(), mEqualities.end());
    out.insert(out.end(), mInequalities.begin(), mInequalities.end());
    return out;
  }

  /**
  */
  void OptimizationProblem::addLinearConstraint(const AffineExpression& expr, ConstraintType type)
  {
    if (type == enEQ)
      mpSolver->addEqCnt(expr, "");
    else 
      mpSolver->addIneqCnt(expr, "");
  }

  /**
  */
  std::vector<double> OptimizationProblem::getCentralFeasiblePoint(const std::vector<double>& x) 
  {
    assert(x.size() == mLowerBounds.size());
    std::vector<double> center(x.size());
    for (size_t i=0; i < x.size(); ++i)
      center[i] = (mLowerBounds[i] + mUpperBounds[i])/2;
    return getClosestFeasiblePoint(center);
  }

  /**
  */
  std::vector<double> OptimizationProblem::getClosestFeasiblePoint(const std::vector<double>& x)
  {
    //LOG_DEBUG("getClosestFeasiblePoint");
    assert(mVariables.size() == x.size());
    QuadraticExpression obj;
    for (size_t i=0; i < x.size(); ++i) 
    {
      obj += QuadraticExpression::square(AffineExpression(mVariables[i]) - x[i]);
    }
    mpSolver->setVarBounds(mVariables, mLowerBounds, mUpperBounds);
    mpSolver->setObjective(obj);
    CvxOptStatus status = mpSolver->optimize();
    if(status != enConvexSubProbSolved) 
    {
      mpSolver->writeToFile("/tmp/fail.lp");
      //PRINT_AND_THROW("couldn't find a feasible point. there's probably a problem with variable bounds (e.g. joint limits). wrote to /tmp/fail.lp");
    }
    return mpSolver->getVarValues(mVariables);
  }

  /**
  */
  std::vector<std::string> OptimizationProblem::getCostNames() const
  {
    std::vector<std::string> result;
    std::transform(mCosts.begin(), mCosts.end(), std::back_inserter(result), [&] (const auto& cost) { return cost->getName(); });
    return result;
  }

  /**
  */
  std::vector<std::string> OptimizationProblem::getConstraintNames() const
  {
    std::vector<std::string> result;
    std::transform(mEqualities.begin(),   mEqualities.end(), std::back_inserter(result), [&] (const auto& cost) { return cost->getName(); });
    std::transform(mInequalities.begin(), mInequalities.end(), std::back_inserter(result), [&] (const auto& cost) { return cost->getName(); });
    return result;
  }
}
}