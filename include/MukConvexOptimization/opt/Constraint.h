#pragma once
#include "opt/Cnt.h"
#include "opt/Variable.h"

#include <memory>
#include <string>
#include <vector>

namespace gris
{
  namespace opt
  {
    class ConvexConstraints;
    class ConvexOptimizationSolver;
     
    /** \brief Non-convex constraint for a convex optimization problem
    */
    class Constraint
    {
      public:
        Constraint() : mName("unnamed") {}
        Constraint(const std::string& name) : mName(name) {}
        virtual ~Constraint() {}

      public:
        const std::string&  getName()                         const { return mName; }
        void                setName(const std::string& name)        { mName = name; }

      public:
        /** \brief Returns the type (equality or inequality) of this constraint*/
        virtual ConstraintType type() const = 0;

        /** \brief Evaluate at solution vector x
          \param  x: the current estimate for the variables of the optimization problem
          \return A vector of no specific size, each element indicating an error.
        */
        virtual std::vector<double> value(const std::vector<double>& x) const = 0;

        /** \brief Convexifies this non-convex constraint at the current estimate x
          \param  x: the current estimate for the variables of the optimization problem
          \param  model: The solver, to which this class might add convexified constraints
          \return New convexified constraints that were added by this class to the solver
        */
        virtual std::shared_ptr<ConvexConstraints> convexify(const std::vector<double>& x, ConvexOptimizationSolver& model) const = 0;

        /** \brief Returns the variables associated with this constraint
        */
        //virtual std::vector<Variable> getVars() const = 0;

      public:
        std::vector<double> violations(const std::vector<double>& x);  /** Calculate constraint violations (positive part for inequality constraint, absolute value for inequality constraint)*/        
        double              violation (const std::vector<double>& x);  /** Sum of violations */        
        
      protected:
        std::string               mName;
    };

    /**
    */
    using ConstraintPtr = std::shared_ptr<Constraint>;

    /**
    */
    class EqConstraint : public Constraint
    {
      public:
        virtual ConstraintType type() const { return enEQ; }
    };

    /**
    */
    class IneqConstraint : public Constraint 
    {
      public:
        virtual ConstraintType type() const { return enINEQ; }
    };
  }
}