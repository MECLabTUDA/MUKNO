#include "private/muk.pch"

#include "PlannerManualLinear.h"

#include "MukCommon/Waypoints.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukProblemDefinition.h"

#include <algorithm>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

namespace gris
{
namespace muk
{
  REGISTER_PLANNER(PlannerManualLinear);

  // -------------------------------------------------

  /**
  */
  PlannerManualLinear::PlannerManualLinear()
    : IPathPlanner()
  {
  }
      
  /**
  */
  PlannerManualLinear::~PlannerManualLinear() 
  {
  }
    
  /**
  */
  MukPath PlannerManualLinear::extractPath(size_t idx) const
  {
    if (idx >= mPaths.size())
    {
      std::string info = "Found paths: " + std::to_string(mPaths.size()) + ". Index requested: " + std::to_string(idx);
      throw MUK_EXCEPTION("Not enough paths available", info.c_str());
    }
    return mPaths[idx];
  }

  /**
  */
  MukPathGraph PlannerManualLinear::extractRoadmap() const
  {
    MukPathGraph ret;
    return ret;
  }
        
  /**
  */
  bool PlannerManualLinear::calculate()
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
    auto waypoints = pProbDef->getWaypoints();
    mPaths.push_back(MukPath());
    mPaths.back().setRadius(pProbDef->getRadius());
    auto& path = mPaths.back().getStates();
    if (waypoints.empty())
    {
      auto pair = std::make_pair<size_t, size_t>(0, 0);
      double minDist = std::numeric_limits<double>::infinity();
      for (size_t i(0); i<initStates.size(); ++i)
      {
        for (size_t j(0); j<goalStates.size(); ++j)
        {
          auto d = (initStates[i].coords-goalStates[j].coords).squaredNorm();
          if (d < minDist)
          {
            pair.first = i;
            pair.second = j;
            minDist = d;
          }
        }
      }
      path.push_back(initStates[pair.first]);
      path.push_back(goalStates[pair.second]);
    }
    else
    {
      size_t idx(0);
      double minDist = std::numeric_limits<double>::infinity();
      for (size_t i(0); i<initStates.size(); ++i)
      {
        auto d = (initStates[i].coords-waypoints.front().coords).squaredNorm();
        if (d < minDist)
        {
          idx = i;
          minDist = d;
        }
      }
      path.push_back(initStates[idx]);
      if (waypoints.size() > 1)
      {
        for (size_t i(1); i<waypoints.size(); ++i)
        {
          path.push_back(waypoints[i]);
        }
      }
      idx = 0;
      minDist = std::numeric_limits<double>::infinity();
      for (size_t i(0); i<goalStates.size(); ++i)
      {
        auto d = (goalStates[i].coords-path.back().coords).squaredNorm();
        if (d < minDist)
        {
          idx = i;
          minDist = d;
        }
      }
      path.push_back(goalStates[idx]);
    }

    path[0].tangent = path[1].coords - path[0].coords;
    path[0].tangent.normalize();
    for (size_t i(1); i<path.size(); ++i)
    {
      path[i].tangent = path[i].coords - path[i-1].coords;
      path[i].tangent.normalize();
    }
    return true;
  }

  /**
  */
  size_t PlannerManualLinear::availablePaths() const
  {
    return mPaths.size();
  }
}
}

