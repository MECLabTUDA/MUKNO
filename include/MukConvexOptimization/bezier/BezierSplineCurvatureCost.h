#pragma once
#include "bezier/BezierOptimizationModel.h"

#include "opt/Cost.h"

namespace gris
{
  namespace bezier
  {
    /** \brief 
    */
    class BezierSplineCurvatureCost : public opt::Cost
    {
      public:
        BezierSplineCurvatureCost(const BezierOptimizationModel& model, double weight = 1.0);

      public:
        void setWeigth(double d)            { mWeight = d;  }
        void setBezierSplineIndex(size_t i) { mIdx = i; }
        void setMaxIndex(size_t idx)        { mMaxIdx = idx; }

      public:
        virtual opt::ConvexObjectivePtr convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const override;
        virtual double value(const std::vector<double>& x) const override;

      private:
        const std::vector<ConstBezierConfigPtr> mConfigs;
        const BezierOptimizationModel& mModel;
        //opt::QuadraticExpression mExpression;
        double mWeight;
        size_t mIdx;
        size_t mMaxIdx;
    };
  }
}
