#include "opt/ConvexOptimizationSolver.h"

namespace gris
{
namespace opt
{
  /**
  */
  Variable ConvexOptimizationSolver::addVar(const std::string& name, double lb, double ub)
  {
    auto v = addVar(name);
    setVarBounds(v, lb, ub);
    return v;
  }

  /**
  */
  void ConvexOptimizationSolver::removeVar(const Variable& var)
  {
    std::vector<Variable> vars(1, var);
    removeVars(vars);
  }

  /**
  */
  void ConvexOptimizationSolver::removeCnt(const Cnt& cnt)
  {
    std::vector<Cnt> cnts(1, cnt);
    removeCnts(cnts);
  }

  /**
  */
  double ConvexOptimizationSolver::getVarValue(const Variable& var) const
  {
    std::vector<Variable> vars(1, var);
    return getVarValues(vars)[0];
  }

  /**
  */
  void ConvexOptimizationSolver::setVarBounds(const Variable& var, double lower, double upper)
  {
    std::vector<double> lowers(1, lower), uppers(1, upper);
    std::vector<Variable> vars(1, var);
    setVarBounds(vars, lowers, uppers);
  }
}
}