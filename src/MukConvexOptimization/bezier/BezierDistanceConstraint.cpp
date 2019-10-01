#include "private/muk.pch"
#include "bezier/BezierDistanceConstraint.h"

namespace gris
{
  namespace bezier
  {
    /**
    */
    BezierDistanceConstraint::BezierDistanceConstraint(const BezierOptimizationModel& model, const muk::ICollisionDetector* pColl)
      : mModel(model)
      , mpColl(pColl)
      , mWeight(1.0)
      , mDistanceThreshold(0.0)
    {
      mName = "BezierDistanceConstraint";
    }

    /**
    */
    opt::ConvexConstraintsPtr BezierDistanceConstraint::convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const
    {
      auto out = std::make_shared<opt::ConvexConstraints>(&model);
      const size_t N = mModel.getStateVariables().rows();
      std::vector<opt::AffineExpression> expressions;
      for (size_t i=0; i < N; ++i) 
      {
        const auto pA = mModel.getWaypoint(i, x);
        Vec3d pB; // the nearest neighbor, an obstacle
        if (mpColl->nearestNeighbor(pA, pB))
        {
          const auto distance = (pA - pB).norm();
          if ( distance < mDistanceThreshold )
          {
            opt::AffineExpression next;
            const auto vars = mModel.getVariables(i); // variables x,y,z of the waypoint i
            const auto n    = (pB-pA).normalized(); // normal towards the obstacle
                                                    // equation 13.1 in Schulman 2014
            const auto grad_x_sd   = -n; // assume Jacobian = identity.
                                         // equation 13.2 in Schulman 2014
            next += distance;
            opt::AffineExpression aff;
            {
              aff.constant() = 0;
              aff.vars()     = { vars[0], vars[1], vars[2] };
              aff.coeffs()   = { grad_x_sd[0], grad_x_sd[1], grad_x_sd[2] };
            }
            next += aff;
            next -= grad_x_sd.dot(pA);
            expressions.push_back(next);
          }
        }
      }
      const auto distPen = opt::AffineExpression(mDistanceThreshold);
      for (size_t i(0); i<expressions.size(); ++i)
      {
        auto viol = distPen - expressions[i];
        out->addIneqCnt(viol);
      }
      return out;
    }

    /**
    */
    std::vector<double> BezierDistanceConstraint::value(const std::vector<double>& x) const
    {
      std::vector<double> vals;
      const size_t N = mModel.getStateVariables().rows();

      for (size_t i=0; i<N; ++i) 
      {
        Vec3d p = mModel.getWaypoint(i, x);
        Vec3d nn;
        if (mpColl->nearestNeighbor(p, nn))
        {
          const auto distance = (p - nn).norm();
          if ( distance < mDistanceThreshold )
          {
            vals.push_back(std::max(0.0, (mDistanceThreshold - distance)));
          }
        }
      }
      return vals;
    }
  }
}