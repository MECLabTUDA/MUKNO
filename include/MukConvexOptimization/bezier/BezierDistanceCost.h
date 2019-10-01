#pragma once
#include "bezier/BezierOptimizationModel.h"

#include "opt/Cost.h"

namespace gris
{
  namespace bezier
  {
    /** \brief A simple quadratic cost function on trajectories, adding up their distance to risk structures
    */
    class BezierDistanceCost : public opt::Cost
    {
      public:
        BezierDistanceCost(const BezierOptimizationModel& model, const muk::ICollisionDetector* pColl);

      public:
        void setDistanceThreshold(double d) { mDistanceThreshold = d;  }
        void setWeigth(double d)            { mWeight = d;  }

      public:
        virtual opt::ConvexObjectivePtr convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const;
        virtual double value(const std::vector<double>& x) const;

      private:
        const muk::ICollisionDetector* mpColl;
        const BezierOptimizationModel& mModel;
        const muk::VariableMatrix mVariables;
        double mWeight;
        double mDistanceThreshold;
    };
  }
}