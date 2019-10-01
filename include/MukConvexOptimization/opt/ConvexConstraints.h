#pragma once
#include "opt/AffineExpression.h"
#include "opt/Cnt.h"

#include <assert.h>
#include <vector>
#include <memory>

namespace gris
{
  namespace opt
  {
    class ConvexOptimizationSolver;

    /** \brief Stores convex inequality constraints and affine equality constraints.

      Actually only affine inequality constraints are currently implemented.
    */
    class ConvexConstraints 
    {
      public:
        ConvexConstraints(ConvexOptimizationSolver* model) 
          : mpModel(model) 
        {}
        ~ConvexConstraints();

        ConvexConstraints() = delete;
        ConvexConstraints(ConvexConstraints&) = delete;
        ConvexConstraints& operator=(const ConvexConstraints&) = delete;

      public:
        void addEqCnt  (const AffineExpression&); /// Expression that should == 0
        void addIneqCnt(const AffineExpression&); /// Expression that should <= 0

        void setModel(ConvexOptimizationSolver* model) { assert( ! inModel() ); mpModel = model; }
        bool inModel() const                            { return mpModel != nullptr; }

        void addConstraintsToModel();
        void removeFromModel();

        std::vector<double> violations(const std::vector<double>& x);
        double              violation (const std::vector<double>& x);

      public:
        std::vector<AffineExpression> mEqualities;
        std::vector<AffineExpression> mInequalities;

      private:
        ConvexOptimizationSolver* mpModel;
        std::vector<Cnt>          mConstraints;
    };

    /**
    */
    using ConvexConstraintsPtr = std::shared_ptr<ConvexConstraints>;
  }
}
