#pragma once
#include "opt/AffineExpression.h"
#include "opt/Constraint.h"
#include "opt/Cost.h"
#include "opt/QuadraticExpression.h"
#include "opt/Variable.h"

namespace gris
{
  namespace opt
  {
    /** \brief Non-convex optimization problem
    */
    class OptimizationProblem 
    {
      public:
        OptimizationProblem();
        virtual ~OptimizationProblem() {}

      public:
        void setSolver(ConvexOptimizationSolverPtr pObj) { mpSolver = pObj; }

      public:
        Variable              createVariable (const std::string& name, double lb, double ub);
        std::vector<Variable> createVariables(const std::vector<std::string>& names); /** create variables with bounds [-INFINITY, INFINITY]  */      
        std::vector<Variable> createVariables(const std::vector<std::string>& names, const std::vector<double>& lb, const std::vector<double>& ub); /** create variables with bounds [lb[i], ub[i] */
        void setLowerBounds(const  std::vector<double>& lb); /** set the lower bounds of all the variables */
        void setUpperBounds(const  std::vector<double>& ub); /** set the upper bounds of all the variables */
        void setLowerBounds(const  std::vector<double>& lb, const  std::vector<Variable>& vars); /** set lower bounds of some of the variables */
        void setUpperBounds(const  std::vector<double>& ub, const  std::vector<Variable>& vars); /** set upper bounds of some of the variables */
      
        /** Note: in the current implementation, this function just adds the constraint to the
        * model. So if you're not careful, you might end up with an infeasible problem. */
        void addLinearConstraint  (const AffineExpression&, ConstraintType type);
        void addCost              (CostPtr); /** Add nonlinear cost function */
        void addConstraint        (ConstraintPtr); /** Add nonlinear constraint function */
        void addEqConstraint      (ConstraintPtr);
        void addIneqConstraint    (ConstraintPtr);

        /** Find closest point to solution  std::vector x that satisfies linear inequality constraints */
        std::vector<double> getCentralFeasiblePoint(const  std::vector<double>& x);
        std::vector<double> getClosestFeasiblePoint(const  std::vector<double>& x);

      public:
        std::vector<ConstraintPtr>          getConstraints()      const;
        const std::vector<CostPtr>&         getCosts()            const { return mCosts; }
        const std::vector<ConstraintPtr>&   getEqConstraints()    const { return mEqualities; }
        const std::vector<ConstraintPtr>&   getIneqConstraints()  const { return mInequalities; }
        const std::vector<double>&          getLowerBounds()      const { return mLowerBounds; }
        const std::vector<double>&          getUpperBounds()      const { return mUpperBounds; }
        ConvexOptimizationSolverPtr         getModel()            const { return mpSolver;}
        const std::vector<Variable>&        getVars()             const { return mVariables;}
        size_t                              getNumCosts()         const { return mCosts.size();}
        size_t                              getNumConstraints()   const { return mEqualities.size() + mInequalities.size(); }
        size_t                              getNumVars()          const { return mVariables.size(); }

      public:
        std::vector<std::string> getCostNames() const;
        std::vector<std::string> getConstraintNames() const;

      protected:
        ConvexOptimizationSolverPtr  mpSolver;
        std::vector<Variable>        mVariables;
        std::vector<double>          mLowerBounds;
        std::vector<double>          mUpperBounds;
        std::vector<CostPtr>         mCosts;
        std::vector<ConstraintPtr>   mEqualities;
        std::vector<ConstraintPtr>   mInequalities;
    };

    using OptimizationProblemPtr = std::shared_ptr<OptimizationProblem>;
  }
}
