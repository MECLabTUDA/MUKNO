#pragma once
#include "opt/ConvexOptimizationSolver.h"

#include <exception>
#include <memory>

namespace gris
{
  namespace opt
  {
    /**
    */
    class GurobiException : public std::exception 
    {
      public:
        GurobiException(const std::string& s1, const std::string& s2, const std::string& details = "");

      public:
        virtual const char* what() const
        {
          return mWhat.c_str();
        }

      private:
        std::string mExpression;
        std::string mFunction;
        std::string mDetails;
        std::string mWhat;
    };

    /**
    */
    class GurobiSolver : public ConvexOptimizationSolver
    {
      public:
        static ConvexOptimizationSolverPtr create();

      public:
        GurobiSolver();
        virtual ~GurobiSolver();

      public:
        virtual void update();
        virtual CvxOptStatus optimize();

      public:
        virtual Variable   addVar     (const std::string& name, double lb, double ub);
        virtual Variable   addVar     (const std::string& name);
        virtual Cnt        addEqCnt   (const AffineExpression&, const std::string& name); // expr == 0
        virtual Cnt        addIneqCnt (const AffineExpression&, const std::string& name); // expr <= 0
        virtual Cnt        addIneqCnt (const QuadraticExpression&, const std::string& name); // expr <= 0

      public:
        virtual void removeVars(const std::vector<Variable>& vars);
        virtual void removeCnts(const std::vector<Cnt>& cnts);
        
      public:
        virtual std::vector<Variable> getVars()                                       const;
        virtual std::vector<double>   getVarValues(const std::vector<Variable>& vars) const;
        virtual void                  setVarBounds(const std::vector<Variable>& vars, const std::vector<double>& lower, const std::vector<double>& upper);

      public:
        virtual void setObjective(const AffineExpression&);
        virtual void setObjective(const QuadraticExpression&);

      public:
        /** Don't use this function, because it adds constraints that aren't tracked  */
        /*CvxOptStatus optimizeFeasRelax();*/

      public:
        virtual void writeToFile(const std::string& fname);
        virtual void setVerbose(bool b);
      
      private:
        // instances share the gurobi environment
        struct S_Impl;
        static std::shared_ptr<S_Impl> s_mp;

        // instances own their own model
        struct Impl;
        std::unique_ptr<Impl> mp;

        std::vector<Variable> mVariables;
        std::vector<Cnt>      mConstraints;
        bool mVerbose = false;
    };
  }
}
