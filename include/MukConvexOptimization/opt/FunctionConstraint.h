#pragma once

#include "opt/Constraint.h"
#include "opt/ConvexConstraints.h"
#include "opt/MatrixOfVector.h"
#include "opt/VectorToVectorFunction.h"

namespace gris
{
  namespace opt
  {
    /** \brief

      A function constraint can be created by either
        - supply both error function and gradient
        - supply error function and obtain derivative numerically 
    */
    class FunctionConstraint : public Constraint 
    {
      public:
        FunctionConstraint(std::shared_ptr<VectorToVectorFunction> f, const std::vector<Variable>& vars, const Eigen::VectorXd& weights, ConstraintType type, const std::string& name);
        virtual ~FunctionConstraint() {}

      public:
        virtual std::vector<double>   value(const std::vector<double>& x) const;
        virtual ConvexConstraintsPtr  convexify(const std::vector<double>& x, ConvexOptimizationSolver& model) const;
        virtual ConstraintType        type()      const { return mType; }
        virtual std::vector<Variable> getVars()   const { return mVars; }

      private:
        Eigen::MatrixXd forwardNumericalJacobian(const VectorToVectorFunction& f, const Eigen::VectorXd& x) const;

      protected:
        VectorToVectorFunctionPtr mFunction;
        MatrixOfVectorPtr     mDerivative;
        std::vector<Variable> mVars;
        Eigen::VectorXd       mWeights;
        ConstraintType        mType;
        double                mEpsilon;
        Eigen::VectorXd       mScaling;
    };
  }
}
