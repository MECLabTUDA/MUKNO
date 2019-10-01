#pragma once
#include "bezier/BezierOptimizationModel.h"

#include "opt/Cost.h"

namespace gris
{
  namespace bezier
  {
    /** \brief A quadratic cost function on 3 waypoints of a Bézier Spline.
    */
    class BezierSplineDistanceCost : public opt::Cost
    {
    public:
      BezierSplineDistanceCost(const BezierOptimizationModel& model, const muk::ICollisionDetector* pColl);

    public:
      void setDistanceThreshold(double d) { mDistanceThreshold = d;  }
      void setWeigth(double d)            { mWeight = d;  }
      void setBezierSplineIndex(size_t i);

    public:
      virtual opt::ConvexObjectivePtr convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const;
      virtual double value(const std::vector<double>& x) const;

    private:
      const muk::ICollisionDetector* mpColl;
      const BezierOptimizationModel& mModel;
      const muk::VariableMatrix mVariables;
      double mWeight;
      double mDistanceThreshold;
      size_t mIdx;
    };
  }
}