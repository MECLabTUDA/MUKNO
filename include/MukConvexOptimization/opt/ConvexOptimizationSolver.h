#pragma once
#include "opt/AffineExpression.h"
#include "opt/Cnt.h"
#include "opt/Constraint.h"
#include "opt/QuadraticExpression.h"
#include "opt/Variable.h"

#include <memory>

namespace gris
{
  namespace opt
  {
    /** brief Result state after a call to optimize a convex Model
    */
    enum CvxOptStatus
    {
      enConvexSubProbSolved,
      enConvexSubProbInfeasible,
      enConvexSubProbFailed
    };

    /** @brief Convex optimization problem

    Gotchas:
    - after adding a variable, need to call update() before doing anything else with that variable
    */
    class ConvexOptimizationSolver
    {
      public:
        virtual ~ConvexOptimizationSolver() {}

      public:
        virtual void update() = 0; // call after adding/deleting stuff
        virtual CvxOptStatus optimize()=0;

      public:
        virtual Variable   addVar     (const std::string& name, double lb, double ub);
        virtual Variable   addVar     (const std::string& name) = 0;
        virtual Cnt        addEqCnt   (const AffineExpression&, const std::string& name) = 0; // expr == 0
        virtual Cnt        addIneqCnt (const AffineExpression&, const std::string& name) = 0; // expr <= 0
        virtual Cnt        addIneqCnt (const QuadraticExpression&, const std::string& name) = 0; // expr <= 0

      public:
        virtual void removeVar(const Variable& var);
        virtual void removeCnt(const Cnt& cnt);
        virtual void removeVars(const std::vector<Variable>& vars) = 0;
        virtual void removeCnts(const std::vector<Cnt>& cnts) = 0;

      public:
        virtual double                getVarValue(const Variable& var)                const;
        virtual std::vector<Variable> getVars()                                       const = 0;
        virtual std::vector<double>   getVarValues(const std::vector<Variable>& vars) const = 0;
        virtual void                  setVarBounds(const std::vector<Variable>& vars, const std::vector<double>& lower, const std::vector<double>& upper) = 0;
        virtual void                  setVarBounds(const Variable& var, double lower, double upper);
      
      public:
        virtual void setObjective(const AffineExpression&)    = 0;
        virtual void setObjective(const QuadraticExpression&) = 0;
      
      public:
        virtual void writeToFile(const std::string& fname)=0;
        virtual void setVerbose(bool b) = 0;
    };

    /**
    */
    using ConvexOptimizationSolverPtr = std::shared_ptr<ConvexOptimizationSolver>;
  }
}
