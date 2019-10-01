#include "private/muk.pch"
#include "bezier/BezierSplineCurvatureCost.h"

namespace
{
  void setName(std::string& str, size_t idx)
  {
    std::stringstream ss;
    ss << "BezierSplineDistanceCost_" << idx;
    str = ss.str();
  }
}

namespace gris
{
  namespace bezier
  {
    /**
    */
    BezierSplineCurvatureCost::BezierSplineCurvatureCost(const BezierOptimizationModel& model, double weight)
      : mModel(model)
      , mWeight(weight)
    {
      ::setName(mName, mIdx);
    }

    /**
    */
    opt::ConvexObjectivePtr BezierSplineCurvatureCost::convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const
    {
      auto result = std::make_shared<opt::ConvexObjective>(&model);
     
      const auto w1 = mModel.getVariables(mIdx);
      const auto w2 = mModel.getVariables(mIdx+1);
      const auto w3 = mModel.getVariables(mIdx+2);
      
      // punish
      //  (1) small distance between w1 and w3
      //  (2) large distance between w2 and P = 0.5*(w1+w3)
      opt::QuadraticExpression expr;
      if (mIdx == 0)
      {
        for (size_t i=0; i<3; ++i)
        {
          auto vel = opt::AffineExpression(w2[i]) + (opt::AffineExpression(w2[i])-opt::AffineExpression(w1[i])) - opt::AffineExpression(w3[i]);
          expr += mWeight * opt::QuadraticExpression::square(vel);
        }
      }
      else if (mIdx == mMaxIdx)
      {
        for (size_t i=0; i<3; ++i)
        {
          auto vel = opt::AffineExpression(w2[i]) + (opt::AffineExpression(w2[i])-opt::AffineExpression(w3[i])) - opt::AffineExpression(w1[i]);
          expr += mWeight * opt::QuadraticExpression::square(vel);
        }
      }
      else
      {
        // (1)
        for (size_t i=0; i<3; ++i)
        {
          auto vel = opt::AffineExpression(w1[i]) - opt::AffineExpression(w3[i]);
          expr += mWeight * opt::QuadraticExpression::square(vel);
        }
        // (2)
        for (size_t i=0; i<3; ++i)
        {
          auto vel = opt::AffineExpression(w2[i]) - 0.5*(opt::AffineExpression(w1[i]) + opt::AffineExpression(w3[i]));
          expr += mWeight * opt::QuadraticExpression::square(vel);
        }
      }
      result->addQuadraticExpression(expr);
      return result;
    }

    /**
    */
    double BezierSplineCurvatureCost::value(const std::vector<double>& x) const
    {
      const auto W1 = mModel.getWaypoint(mIdx, x);
      const auto W2 = mModel.getWaypoint(mIdx+1, x);
      const auto W3 = mModel.getWaypoint(mIdx+2, x);

      auto val = 0.0;
      if (mIdx == 0)
      {
        for (size_t i=0; i<3; ++i)
        {
          auto vel = W2[i] + W2[i] - W1[i] - W3[i];
          val += mWeight *vel*vel;
        }
      }
      else if (mIdx == mMaxIdx)
      {
        for (size_t i=0; i<3; ++i)
        {
          auto vel = W2[i] + W2[i] - W3[i] - W1[i];
          val += mWeight *vel*vel;
        }
      }
      else
      {
        // (1)
        for (size_t i=0; i<3; ++i)
        {
          auto vel = W1[i] - W3[i];
          val += mWeight *vel*vel;
        }
        // (2)
        for (size_t i=0; i<3; ++i)
        {
          auto vel = W2[i] - 0.5*(W1[i] + W3[i]);
          val += mWeight * vel*vel;
        }
      }
      return val;
    }
  }
}