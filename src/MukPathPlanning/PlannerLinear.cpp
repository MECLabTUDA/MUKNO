#include "private/muk.pch"
#include "PlannerLinear.h"

#include "MukCommon/muk_eigen_helper.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukProblemDefinition.h"
#include "MukCommon/Waypoints.h"

#include <Eigen/Dense>

#include <boost/format.hpp>

#include <omp.h>

namespace 
{
  using namespace gris::muk;
}

namespace gris
{
  namespace muk
  {
    REGISTER_PLANNER(PlannerLinear);

    /**
    */
    PlannerLinear::PlannerLinear()
      : IPathPlanner()
      , mMaxDistToGoal(0.5)
      , mResolution(0.1)
    {
      declareProperty<double>("MaxDistToGoal"
        , [&] (double d)        { setMaxDistToGoal(d); }
        , [&] ()                { return getMaxDistToGoal(); });
      declareProperty<double>("Resolution"
        , [&] (double d)        { setResolution(d); }
        , [&] ()                { return getResolution(); });
    }

    /**
    */
    PlannerLinear::~PlannerLinear()
    {
    }

    /**
    */
    MukPath PlannerLinear::extractPath(size_t idx) const
    {
      if (idx >= mFoundPaths.size())
      {
        std::string info = "Found paths: " + std::to_string(mFoundPaths.size()) + ". Index requested: " + std::to_string(idx);
        throw MUK_EXCEPTION("Not enough paths available", info.c_str());
      }
      return mFoundPaths[idx];
    }

    /**
    */
    MukPathGraph PlannerLinear::extractRoadmap() const
    {
      MukPathGraph ret;
      return ret;
    }

    /**
    */
    bool PlannerLinear::calculate()
    {
      auto pProbDef = mpProbDef.lock();
      if (!pProbDef)
      {
        throw MUK_EXCEPTION_SIMPLE("no problem definition available");
      }
      auto initStates = pProbDef->getStartStates();
      auto goalStates = pProbDef->getGoalStates();
      if (initStates.empty() || goalStates.empty())
      {
        throw MUK_EXCEPTION_SIMPLE("no initial or goal states");
      }
      LOG_LINE << "# initial states: " << initStates.size();
      LOG_LINE << "# goal states:    " << goalStates.size();
      std::vector<MukPath> work;
      // collect all paths, that reach the goal area
      for (const auto& state : goalStates)
      {
        using namespace Eigen;
        Vector3d p = asEigen(state.coords);
        Vector3d t = asEigen(state.tangent);
        auto line = ParametrizedLine<double, 3>(p ,t);
        double minDist = std::numeric_limits<double>::infinity();
        double maxLength = 0;
        bool valid = false;
        size_t idx(0);
        // determine the goal state that achieves the longest path
        for (size_t i(0); i<initStates.size(); ++i)
        {
          const auto& goalstate = initStates[i];
          Vector3d q = asEigen(goalstate.coords);
          auto goalNN = line.projection(q);
          const auto length = (p-goalNN).squaredNorm();
          const auto dist   = (goalNN-q).norm();
          if ( dist < mMaxDistToGoal && length > maxLength)
          {
            valid = true;
            idx = i;
            maxLength = length;
            minDist = dist;
          }
        }
        if (valid)
        {
          MukPath next;
          next.setRadius(pProbDef->getRadius());
          auto startNN = line.projection(asEigen(initStates[idx].coords));
          const auto p = asMuk(startNN);
          const auto t = (state.coords - p).normalized();
          auto start   = MukState(p,t);
          next.getStates().push_back(start);
          next.getStates().push_back(state);
          work.push_back(next);
        }
      }
      mFoundPaths.resize(work.size());
      const double maxDist = pProbDef->getSafetyDist()+pProbDef->getRadius();
 #pragma omp parallel for
      for (int i(0); i<(int) work.size(); ++i)
      {
        double stepSize = mResolution;
        const auto& path = work[i].getStates();
        const auto p = path.front().coords;
        const auto t = path.front().tangent;
        double length = (path.back().coords-p).norm();
        auto N = static_cast<size_t>( length / stepSize + 0.5 );
        size_t j(0);
        bool valid = true;
        do
        {
          Vec3d q = p + (j*stepSize)*t;
          Vec3d nn;
          if (mpCollisionDetector->nearestNeighbor(q, nn) && (q-nn).norm() < maxDist)
          {
            valid = false;
          }
          ++j;
        }
        while(valid && j<N);
        if (valid)
        {
          mFoundPaths[i].setRadius(pProbDef->getRadius());
          mFoundPaths[i].getStates() = path;
        }
      }

      mFoundPaths.erase(std::remove_if(mFoundPaths.begin(), mFoundPaths.end(), [&] (const auto& path) { return path.getStates().empty(); }), mFoundPaths.end());
      if (mFoundPaths.empty())
        return false;
      else
        return true;
    }

    /**
    */
    size_t PlannerLinear::availablePaths() const
    {
      return mFoundPaths.size();
    }
  }
}