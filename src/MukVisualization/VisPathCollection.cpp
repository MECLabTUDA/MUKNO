#include "private/muk.pch"
#include "VisPathCollection.h"

#include "MukCommon/MukException.h"
#include "MukCommon/PathCollection.h"

#include <boost/format.hpp>

namespace gris
{
namespace muk
{
  /**
  */
  VisPathCollection::VisPathCollection(const std::string& name)
    : mName(name)
  {
  }

  /**
  */
  VisPathCollection::~VisPathCollection()
  {
  }

  /**
  */
  void VisPathCollection::addStartRegion(std::shared_ptr<VisStateRegion> pRegion)
  {
    mStartRegions.push_back(pRegion);
  }

  /**
  */
  void VisPathCollection::addGoalRegion(std::shared_ptr<VisStateRegion> pRegion)
  {
    mGoalRegions.push_back(pRegion);
  }

  /**
  */
  /*void VisPathCollection::addWaypoint(std::shared_ptr<VisStateRegion> pRegion)
  {
    mpWaypoints.push_back(pRegion);
  }*/

  /**
  */
  VisStateRegion& VisPathCollection::getStartRegion(size_t i) const
  {
    if (i >= mStartRegions.size())
    {
      std::string err = "Start Region at index " + std::to_string(i) + " does not exist";
      std::string info = "Current number of start regions available: " + std::to_string(mStartRegions.size());
      throw MUK_EXCEPTION(err.c_str(), info.c_str());
    }
    return *mStartRegions[i];
  }

  /**
  */
  VisStateRegion& VisPathCollection::getGoalRegion(size_t i) const
  {
    if (i >= mGoalRegions.size())
    {
      std::string err = "Goal Region at index " + std::to_string(i) + " does not exist";
      std::string info = "Current number of goal regions available: " + std::to_string(mGoalRegions.size());
      throw MUK_EXCEPTION(err.c_str(), info.c_str());
    }
    return *mGoalRegions[i];
  }
  
  /**
  */
  void VisPathCollection::deleteStartRegion(size_t i)
  {
    if (mStartRegions.size() <= i)
    {
      std::string err = "Start Region at index " + std::to_string(i) + " does not exist";
      std::string info = "Current number of start regions available: " + std::to_string(mStartRegions.size());
      throw MUK_EXCEPTION(err.c_str(), info.c_str());
    }
    mStartRegions.erase(mStartRegions.begin() + i);
  }

  /**
  */
  void VisPathCollection::deleteGoalRegion(size_t i)
  {
    if (mGoalRegions.size() <= i)
    {
      std::string err = "Goal Region at index " + std::to_string(i) + " does not exist";
      std::string info = "Current number of goal regions available: " + std::to_string(mGoalRegions.size());
      throw MUK_EXCEPTION(err.c_str(), info.c_str());
    }
    mGoalRegions.erase(mGoalRegions.begin() + i);
  }

  /**
  */
  /*void VisPathCollection::deleteWaypoint(size_t i)
  {
    if (mpWaypoints.size() <= i)
    {
      std::string err = "Waypoint at index " + std::to_string(i) + " does not exist";
      std::string info = "Current number of waypoints available: " + std::to_string(mpWaypoints.size());
      throw MUK_EXCEPTION(err.c_str(), info.c_str());
    }
    mpWaypoints.erase(mpWaypoints.begin() + i);
  }*/
  
  /**
  */
  std::shared_ptr<VisMukPath> VisPathCollection::getMukPath(size_t idx)
  {
    if (mPaths.size() <= idx)
      throw MUK_EXCEPTION("Index bigger than number of Paths", (boost::format("PathCollection '%s' has only %d paths. Passed idx: %d") % mName % mPaths.size() % idx).str().c_str());
    return mPaths[idx];
  }

  /**
  */
  void VisPathCollection::addMukPath(std::shared_ptr<VisMukPath> pVis)
  {
    mPaths.push_back(pVis);
  }

  /**
  */
  void VisPathCollection::deleteMukPath(size_t idx)
  {
    if (idx<mPaths.size())
    {
      mPaths.erase(mPaths.begin()+idx);
    }
    else
    {
      throw MUK_EXCEPTION((boost::format("unable to delete visualization of path with idx %d") % idx).str().c_str(), (boost::format("There are only %d visualizations in PathCollection '%s'") % mPaths.size() % mName).str().c_str());
    }
  }
}
}