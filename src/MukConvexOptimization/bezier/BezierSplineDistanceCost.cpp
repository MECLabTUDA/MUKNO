#include "private/muk.pch"
#include "bezier/BezierSplineDistanceCost.h"

#include "MukPathPlanning/BezierSpiralInterpolator.h"

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
    BezierSplineDistanceCost::BezierSplineDistanceCost(const BezierOptimizationModel& model, const muk::ICollisionDetector* pColl)
      : mModel(model)
      , mpColl(pColl)
      , mWeight(1.0)
      , mDistanceThreshold(0.0)
      , mIdx(0)
    {
      ::setName(mName, mIdx);
    }

    /**
    */
    void BezierSplineDistanceCost::setBezierSplineIndex(size_t i)
    {
      mIdx = i;
      ::setName(mName, mIdx);
    }

    /**
    */
    opt::ConvexObjectivePtr BezierSplineDistanceCost::convexify(const std::vector<double>& x, opt::ConvexOptimizationSolver& model) const
    {
      auto result = std::make_shared<opt::ConvexObjective>(&model);
      // compute the spline.
      const auto W1 = mModel.getWaypoint(mIdx, x);
      const auto W2 = mModel.getWaypoint(mIdx+1, x);
      const auto W3 = mModel.getWaypoint(mIdx+2, x);
      using namespace muk;
      BezierSpiralInterpolator builder;
      BezierSpline spline;
      const auto B0 = 0.5*(W1+W2);
      const auto E0 = 0.5*(W2+W3);
      builder.interpolate(B0,W2,E0, spline);
      // sample and search for closest point
      std::vector<Vec3d> v;
      spline.uniformSamples(0.1, v);
      std::vector<double> dists;
      for(const auto& p : v)
      {
        Vec3d nn;
        mpColl->nearestNeighbor(p, nn);
        dists.push_back((p-nn).squaredNorm());
      }
      auto iter = std::min(v.begin(), v.end());
      auto idx  = std::distance(v.begin(), iter);
      // compute expressions
      std::vector<opt::AffineExpression> expressions;
      Vec3d pB; // the nearest neighbor, an obstacle
      mpColl->nearestNeighbor(v[idx], pB);
      {
        auto addNext = [&] (size_t idx, const Vec3d& waypoint)
        {
          opt::AffineExpression exp;
          Vec3d pB; // the nearest neighbor, an obstacle
          mpColl->nearestNeighbor(waypoint, pB);
          const auto distance = (waypoint - pB).norm();
          if ( distance < mDistanceThreshold )
          {
            const auto vars = mModel.getVariables(idx); // variables x,y,z of the waypoint i
            const auto n    = (pB-waypoint).normalized(); // normal towards the obstacle
            //const auto n    = (pB-v[idx]).normalized(); // normal towards the obstacle
            // equation 13.1 in Schulman 2014
            const auto grad_x_sd = -n; // assume Jacobian = identity.
                                       // equation 13.2 in Schulman 2014
            exp += distance;
            opt::AffineExpression aff;
            {
              aff.constant() = 0;
              aff.vars()     = { vars[0], vars[1], vars[2] };
              aff.coeffs()   = { grad_x_sd[0], grad_x_sd[1], grad_x_sd[2] };
            }
            exp += aff;
            exp -= grad_x_sd.dot(waypoint);
            expressions.push_back(exp);
          }
        };
        //addNext(mIdx, W1);
        addNext(mIdx+1, W2);
        //addNext(mIdx+2, W3);
      }
      
      const auto distPen = opt::AffineExpression(mDistanceThreshold);    
      for (size_t i(0); i<expressions.size(); ++i)
      {
        auto viol = distPen - expressions[i];
        result->addHinge(viol, mWeight);
      }
      return result;
    }

    /**
    */
    double BezierSplineDistanceCost::value(const std::vector<double>& x) const
    {
      double val(0.0);
      
      const auto W1 = mModel.getWaypoint(mIdx, x);
      const auto W2 = mModel.getWaypoint(mIdx+1, x);
      const auto W3 = mModel.getWaypoint(mIdx+2, x);
      using namespace muk;
      BezierSpiralInterpolator builder;
      BezierSpline spline;
      const auto B0 = 0.5*(W1+W2);
      const auto E0 = 0.5*(W2+W3);
      builder.interpolate(B0,W2,E0, spline);
      // sample and search for closest point
      std::vector<Vec3d> v;
      spline.uniformSamples(0.1, v);
      std::vector<double> dists;
      for(const auto& p : v)
      {
        Vec3d nn;
        mpColl->nearestNeighbor(p, nn);
        dists.push_back((p-nn).squaredNorm());
      }
      auto iter = std::min(v.begin(), v.end());
      auto idx  = std::distance(v.begin(), iter);

      Vec3d nn;
      //mpColl->nearestNeighbor(v[idx], nn);
      mpColl->nearestNeighbor(W2, nn);
      {
        auto distance = (W1 - nn).norm();
        /*if ( distance < mDistanceThreshold )
          val += std::max(0.0, (mDistanceThreshold - distance)) * mWeight;*/
        distance = (W2 - nn).norm();
        //if ( distance < mDistanceThreshold )
          val += std::max(0.0, (mDistanceThreshold - distance)) * mWeight;
        /*distance = (W3 - nn).norm();
        if ( distance < mDistanceThreshold )
          val += std::max(0.0, (mDistanceThreshold - distance)) * mWeight;*/
      }
      return val;
    }
  }
}