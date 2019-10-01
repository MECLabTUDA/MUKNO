#pragma once
#include "opt/OptimizationProblem.h"

#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace gris
{
  namespace opt
  {
    /**
    */
    enum EnOptimizerResult
    {
      enOptimizerConverged = 0,
      enOptimizerFailed,
      enBadInitialization,
      enSCOIterationLimitReached,
      enPenaltyIterationLimitReached,
      enOptimizerResult_SIZE
    };

    /**
    */
    static const char* EnOptimizerResultNames[enOptimizerResult_SIZE] =
    {
      "CONVERGED",
      "FAILED",
      "BAD_INITIALIZATION",
      "SCO_ITERATION_LIMIT",
      "PENALTY_ITERATION_LIMIT",
    };
    
    /**
    */
    struct OptResults 
    {
      OptResults() { clear(); }

      void clear();
      
      std::vector<double> x; // solution estimate
      bool                improved;
      EnOptimizerResult   status;
      double              total_cost;
      std::vector<double> cost_vals;
      std::vector<double> cnt_viols;
      int                 n_func_evals;
      int                 n_qp_solves;
    };

    /**
    */
    std::ostream& operator<<(std::ostream& o, const OptResults& r);

    /** Solves an optimization problem
    */
    class Optimizer 
    {
      public:
        using Callback = std::function<void(OptimizationProblem*, std::vector<double>&)>;

      public:
        virtual ~Optimizer() {}

      public:
        virtual EnOptimizerResult optimize() = 0;
        virtual void        setProblem(OptimizationProblemPtr& prob) { mpOptProb = prob; }

      public:
        void                 initialize(const std::vector<double>& x);
        std::vector<double>& x()                                   { return mResults.x; }
        OptResults&          results()                             { return mResults; }
        void                 addCallback(const Callback& f); // called before each iteration

      protected:
        void                 callCallbacks(std::vector<double>& x);

      protected:
        std::vector<Callback>  mCallbacks;
        OptimizationProblemPtr mpOptProb;
        OptResults             mResults;
    };

  }
}
