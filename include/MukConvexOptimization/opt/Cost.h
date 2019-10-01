#pragma once
#include "ConvexObjective.h"
#include "ConvexOptimizationSolver.h"

#include <string>
#include <vector>
#include <memory>

namespace gris
{
  namespace opt
  {
    /** \brief Non-convex cost function, which knows how to calculate its convex approximation (convexify() method)
    */
    class Cost
    {
      public:
        Cost()                        : mName("unnamed") {}
        Cost(const std::string& name) : mName(name) {}
        virtual ~Cost() {}

      public:
        virtual double                value     (const std::vector<double>& v)                                  const = 0; /** Evaluate at solution  std::vector x*/      
        virtual ConvexObjectivePtr    convexify (const std::vector<double>& x, ConvexOptimizationSolver& model) const = 0; /** Convexify at solution  std::vector x*/      
//        virtual std::vector<Variable> getVars()                                                                 const = 0; /** Get problem variables associated with this cost */

      public:
        const std::string&  getName()                         const { return mName; }
        void                setName(const std::string& name)        { mName=name; }

      protected:
        std::string mName;
    };
    
    /**
    */
    using CostPtr = std::shared_ptr<Cost>;
  }
}