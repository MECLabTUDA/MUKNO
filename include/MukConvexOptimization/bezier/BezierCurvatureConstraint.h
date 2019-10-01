#pragma once
#include "bezier/BezierOptimizationModel.h"

#include "opt/VectorToVectorFunction.h"

namespace gris
{
  namespace bezier
  {
    /** \brief If curvature is too large, returns an error function that smoothes the spline.
    */
    struct BezierCurvatureConstraint : public opt::VectorToVectorFunction
    {
      public:
        BezierCurvatureConstraint(const BezierOptimizationModel& model, size_t idx);

      public:
        virtual Eigen::VectorXd operator() (const Eigen::VectorXd& x) const;

      public:
        const std::vector<opt::Variable>& getVariables() { return mVariables; }
        size_t getIndex()               const { return mIdx; }
        void   setMaxIndex(size_t idx)        { mMaxIdx = idx; }

      private:
        const BezierOptimizationModel& mModel;
        std::vector<opt::Variable>     mVariables;  // the nine variables, 3 for each waypoint
        size_t mIdx;                                // the index of a waypoint triple (W_i, W_i+1, W_i+2))
        size_t mMaxIdx;
    };
  }
}
