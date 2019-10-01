#pragma once
#include "AffineExpression.h"
#include "Cnt.h"
#include "ConvexOptimizationSolver.h"
#include "QuadraticExpression.h"

#include <memory>

namespace gris
{
  namespace opt
  {
    /** \brief Stores convex terms in an objective

      For non-quadratic terms like hinge(x) and abs(x), it needs to add auxilliary variables and linear constraints to the model
      Note: When this object is deleted, the constraints and variables it added to the model are removed
    */
    class ConvexObjective 
    {
      public:
        explicit ConvexObjective(ConvexOptimizationSolver* pSolver) : mpSolver(pSolver) {}
        ~ConvexObjective();

        ConvexObjective() = delete;
        ConvexObjective(ConvexObjective&) = delete;
        ConvexObjective& operator=(const ConvexObjective&) = delete;

      public:
        void addAffineExpression (const AffineExpression& expr);
        void addQuadraticExpression(const QuadraticExpression& expr);
        void addHinge   (const AffineExpression&, double coeff);
        void addAbs     (const AffineExpression&, double coeff);
        void addHinges  (const std::vector<AffineExpression>& v);
        void addL1Norm  (const std::vector<AffineExpression>& v);
        void addL2Norm  (const std::vector<AffineExpression>& v);
        void addMax     (const std::vector<AffineExpression>& v);

      public:
        double value(const std::vector<double>& x);
        bool hasSolver()                const { return mpSolver != nullptr; }
        void addConstraintsToModel();
        void removeFromSolver();
      
      public:
        ConvexOptimizationSolver*  mpSolver = nullptr;
        QuadraticExpression        mQuad;
        std::vector<Variable>      mVariables;
        std::vector<AffineExpression> mEqualities;
        std::vector<AffineExpression> mInequalities;
        std::vector<Cnt>           mConstraints;
    };

    /**
    */
    using ConvexObjectivePtr = std::shared_ptr<ConvexObjective>;   
  }
}