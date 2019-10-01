#include "private/muk.pch"
#include "bezier/BezierSplineCurvatureConstraint.h"

namespace gris
{
  namespace bezier
  {
    /**
    */
    BezierSplineCurvatureConstraint::BezierSplineCurvatureConstraint(const BezierOptimizationModel& model, size_t idx)
      : mModel(model)
      , mIdx(idx)
    {
      std::stringstream ss;
      ss << "BezierSplineCurvatureConstraint" << "_" << idx;
      mName = ss.str();
    }

    /**
    */
    opt::ConvexConstraintsPtr BezierSplineCurvatureConstraint::convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const
    {
      auto result = std::make_shared<opt::ConvexConstraints>(&model);
      
      std::vector<opt::AffineExpression> expressions;
      
      const auto w1 = mModel.getWaypoint(mIdx, x);
      const auto w2 = mModel.getWaypoint(mIdx+1, x);
      const auto w3 = mModel.getWaypoint(mIdx+2, x);

      const auto kappa = mModel.getConfigs()[mIdx]->getBuilder().computeKappaMax(w1,w2,w3);;
      if (kappa < mModel.getKappaMax())
      {
        // no constraint
        return result;
      }
      else
      {
        // compute proper places for waypoints
        const auto d1     = (w1 - w2).norm();
        const auto d2     = (w3 - w2).norm();
        const auto center = 0.5*(w1 + w3);
        const auto v      = (w3 - w1).normalized();
        const auto w2New  = 0.5 * (w2 + center);
        const auto w1New  = w2New - d1*v;
        const auto w3New  = w2New + d2*v;

        auto vars = mModel.getVariables(mIdx); // variables x,y,z of the waypoint i
        for (size_t i(0); i<3; ++i)
        {
          expressions.push_back(opt::AffineExpression(vars[i]) - opt::AffineExpression(w1New[i]));
        }
        vars = mModel.getVariables(mIdx+1); // variables x,y,z of the waypoint i
        for (size_t i(0); i<3; ++i)
        {
          expressions.push_back(opt::AffineExpression(vars[i]) - opt::AffineExpression(w2New[i]));
          
        }
        vars = mModel.getVariables(mIdx+2); // variables x,y,z of the waypoint i
        for (size_t i(0); i<3; ++i)
        {
          expressions.push_back(opt::AffineExpression(vars[i]) - opt::AffineExpression(w3New[i]));
        }

        //const auto distPen = opt::AffineExpression(0.0);
        for (size_t i(0); i<expressions.size(); ++i)
        {
          auto viol = expressions[i];
          result->addIneqCnt(viol);
        }
        return result;
      }
    }

    /**
    */
    std::vector<double> BezierSplineCurvatureConstraint::value(const std::vector<double>& x) const
    {
      std::vector<double> result;
      const auto w1 = mModel.getWaypoint(mIdx, x);
      const auto w2 = mModel.getWaypoint(mIdx+1, x);
      const auto w3 = mModel.getWaypoint(mIdx+2, x);
      const auto kappa = mModel.getConfigs()[mIdx]->getBuilder().computeKappaMax(w1,w2,w3);;
      if (kappa < mModel.getKappaMax())
      {
        // no constraint
        return result;
      }
      else
      {
        // compute proper places for waypoints
        const auto d1     = (w1 - w2).norm();
        const auto d2     = (w3 - w2).norm();
        const auto center = 0.5*(w1 + w3);
        const auto v      = (w3 - w1).normalized();
        const auto w2New  = 0.5 * (w2 + center);
        const auto w1New  = w2New - d1*v;
        const auto w3New  = w2New + d2*v;

        result.push_back( (w1-w1New).norm() );
        result.push_back( (w1-w1New).norm() );
        result.push_back( (w1-w1New).norm() );
        result.push_back( (w2-w2New).norm() );
        result.push_back( (w2-w2New).norm() );
        result.push_back( (w2-w2New).norm() );
        result.push_back( (w3-w3New).norm() );
        result.push_back( (w3-w3New).norm() );
        result.push_back( (w3-w3New).norm() );
      }
      return result;
    }
  }
}