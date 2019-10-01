#include "private/muk.pch"
#include "gris_math.h"
#include "MukProblemDefinition.h"
#include "MukException.h"
#include "MukStateRegion.h"
#include "StateRegionSingleDirection.h"

#include <boost/serialization/vector.hpp>
#include <boost/serialization/unique_ptr.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

namespace gris
{
namespace muk
{
  /**
  */
  MukProblemDefinition::MukProblemDefinition()
    : mRadius(0.5)
    , mSafetyDist(0.5)
    , mKappa(0.05)
    , mGoalThreshold(1.0)
    , mGoalAngleThreshold(M_Pi_4)
  {
    appendProperties();
  }

  /**
  */
  MukProblemDefinition::MukProblemDefinition(MukProblemDefinition&& o)
  {
    if (this != &o)
    {
      mRadius = o.mRadius;
      mSafetyDist = o.mSafetyDist;
      mKappa      = o.mKappa;
      mGoalThreshold = mGoalThreshold;
      mGoalAngleThreshold = o.mGoalAngleThreshold;

      mStart = std::move(o.mStart);
      mGoal  = std::move(o.mGoal);
      mWaypoints = std::move(o.mWaypoints);
      
      appendProperties();
    }
  }

  /**
  */
  void MukProblemDefinition::appendProperties()
  {
    declareProperty<double>("Radius",
      std::bind(&MukProblemDefinition::setRadius, this, std::placeholders::_1), 
      std::bind(&MukProblemDefinition::getRadius, this));
    declareProperty<double>("SafetyDistance",
      std::bind(&MukProblemDefinition::setSafetyDist, this, std::placeholders::_1), 
      std::bind(&MukProblemDefinition::getSafetyDist, this));
    declareProperty<double>("Kappa",
      std::bind(&MukProblemDefinition::setKappa, this, std::placeholders::_1), 
      std::bind(&MukProblemDefinition::getKappa, this));
    declareProperty<double>("GoalThreshold",
      std::bind(&MukProblemDefinition::setGoalThreshold, this, std::placeholders::_1), 
      std::bind(&MukProblemDefinition::getGoalThreshold, this));
    declareProperty<double>("GoalAngleThreshold",
      std::bind(&MukProblemDefinition::setGoalAngleThreshold, this, std::placeholders::_1), 
      std::bind(&MukProblemDefinition::getGoalAngleThreshold, this));
    declareProperty<Vec3d>("Bounds-Min"
      , [&] (const Vec3d& p) { mBounds.setMin(p); }
      , [&] ()               { return mBounds.getMin(); });
    declareProperty<Vec3d>("Bounds-Max"
      , [&] (const Vec3d& p) { mBounds.setMax(p); }
      , [&] ()               { return mBounds.getMax(); });
  }


  /**
  */
  void MukProblemDefinition::setRadius(double val)
  {
    mRadius = val;
  }

  /**
  */
  void MukProblemDefinition::setSafetyDist(double val)
  { 
    mSafetyDist = val;
  }

  /**
  */
  void MukProblemDefinition::setKappa(double val)
  { 
    mKappa = val;
  }

  /**
  */
  void MukProblemDefinition::setGoalThreshold(double val)
  {
    mGoalThreshold = val;
  }

  /**
  */
  void MukProblemDefinition::setGoalAngleThreshold(double val)
  {
    mGoalAngleThreshold = val;
  }

  /**
  */
  std::vector<MukState> MukProblemDefinition::getStartStates() const
  {
    std::vector<MukState> ret;
    for (const auto& pRegion : mStart)
    {
      auto states = pRegion->getStates();
      std::copy(states.begin(), states.end(), std::back_inserter(ret));
    }
    return ret;
  }

  /**
  */
  std::vector<MukState> MukProblemDefinition::getGoalStates()  const
  {
    std::vector<MukState> ret;
    for (const auto& pRegion : mGoal)
    {
      auto states = pRegion->getStates();
      std::copy(states.begin(), states.end(), std::back_inserter(ret));
    }
    return ret;
  }

  /**
  */
  std::vector<MukState> MukProblemDefinition::getWaypoints()   const
  {
    return mWaypoints;
  }

  /**
  */
  void MukProblemDefinition::addStartRegion(std::unique_ptr<IStateRegion> mRegion)
  {
    mStart.push_back(std::move(mRegion));
  }

  /**
  */
  void MukProblemDefinition::addGoalRegion (std::unique_ptr<IStateRegion> mRegion)
  {
    mGoal.push_back(std::move(mRegion));
  }

  /**
  */
  IStateRegion& MukProblemDefinition::getStartRegion(size_t i) const
  {
    if (mStart.size() <= i)
    {
      std::string err = "MukProblemDefinition does not have a Start Region at index " + std::to_string(i);
      std::string info = "Number of Start Regions available: " + std::to_string(mStart.size());
      throw MUK_EXCEPTION(err.c_str(), info.c_str());
    }
    return *mStart[i];
  }

  /**
  */
  IStateRegion& MukProblemDefinition::getGoalRegion(size_t i) const
  {
    if (mGoal.size() <= i)
    {
      std::string err = "MukProblemDefinition does not have a Start Region at index " + std::to_string(i);
      std::string info = "Number of Start Regions available: " + std::to_string(mGoal.size());
      throw MUK_EXCEPTION(err.c_str(), info.c_str());
    }
    return *mGoal[i];
  }

  /**
  */
  void MukProblemDefinition::removeWaypoint(size_t index)
  {
    if (mWaypoints.size() > index)
      mWaypoints.erase(mWaypoints.begin() + index);
  }

  /**
  */
  void MukProblemDefinition::insertWaypoint(const MukState& s, size_t index)
  {
    if (mWaypoints.empty())
    {
      if (index==0)
        mWaypoints.push_back(s);
      else
      {
        std::string info = "index: " + std::to_string(index);
        throw MUK_EXCEPTION("Invalid index for vector of this size", info.c_str());
      }
    }
    else
    {
      mWaypoints.insert(mWaypoints.begin()+index, s);
    }
  }

  /**
  */
  void MukProblemDefinition::clearStart()
  {
    mStart.clear();
  }

  /**
  */
  void MukProblemDefinition::clearWaypoints()
  {
    mWaypoints.clear();
  }

  /**
  */
  void MukProblemDefinition::clearGoal()
  {
    mGoal.clear();
  }

  ///**
  //*/
  void MukProblemDefinition::save(const std::string& filename, unsigned int version) const
  {
    std::ofstream ofs(filename);
    boost::archive::text_oarchive oa(ofs);
    const_cast<MukProblemDefinition&>(*this).serialize(oa, version);
  }

  /**
  */
  void MukProblemDefinition::load(const std::string& filename, unsigned int version)
  {
    std::ifstream ifs(filename);
    boost::archive::text_iarchive ia(ifs);
    serialize(ia, version);
  }

  /** \brief Copy the parameters (radius etc..) but not the start and goal states
      
      used to easily create a similar path collection, e.g. as in the case of replanning.
  */
  void MukProblemDefinition::copyParameters(const MukProblemDefinition& other)
  {
    mRadius = other.mRadius;
    mSafetyDist = other.mSafetyDist;
    mKappa = other.mKappa;
    mGoalThreshold = other.mGoalThreshold;
    mGoalAngleThreshold = other.mGoalAngleThreshold;
    mBounds = other.mBounds;
  }

  /** \brief clones every member into target, thereby creating new state regions.

    used to easily create a similar path collection, e.g. as in the case of replanning.
  */
  void MukProblemDefinition::clone(MukProblemDefinition& target) const
  {
    target.copyParameters(*this);
    target.mWaypoints = mWaypoints;
    for(const auto& p : mStart)
    {
      auto pNew = p->clone();
      target.mStart.push_back(std::move(pNew));
    }
    for(const auto& p : mGoal)
    {
      auto pNew = p->clone();
      target.mGoal.push_back(std::move(pNew));
    }
  }

  /**
  */
  template<>
  void MukProblemDefinition::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive& ar, const unsigned version)
  {
    Vec3d min;
    Vec3d max;
    ar & mRadius & mSafetyDist & mKappa & mGoalThreshold & mGoalAngleThreshold & min & max & mStart & mWaypoints & mGoal;
    mBounds.setMin(min);
    mBounds.setMax(max);
  }

  /**
  */
  template<>
  void MukProblemDefinition::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive& ar, const unsigned version)
  {
    ar & mRadius & mSafetyDist & mKappa & mGoalThreshold & mGoalAngleThreshold & mBounds.getMin() & mBounds.getMax() & mStart & mWaypoints & mGoal;
  }
}
}
