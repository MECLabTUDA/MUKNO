#pragma once

#include "private/muk.pch"
#include "INavigator.h"

// MukCommon
#include "MukException.h"
#include "INavigationSupervisor.h"
#include "InterpolatorFactory.h"
#include "overload_deduction.h"

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/join.hpp>

namespace gris
{
  namespace muk
  {
    const double INavigator::POSITION_RESOLUTION_FACTOR = 10;

    NavigatorProperty::NavigatorProperty(INavigator* nav)
      : gstd::DynamicProperty()
    {
      for (std::size_t i = 1; i <= INavigator::NavigatorFeatureCount(); ++i)
      {
        declareProperty<bool>(INavigator::NavigatorFeatureToString(INavigator::NavigatorFeature(i)),
          std::bind(&INavigator::setNavigatorFeature, nav, INavigator::NavigatorFeature(i), std::placeholders::_1),
          std::bind(&INavigator::isFeatureActivated, nav, INavigator::NavigatorFeature(i)));
      }

      declareProperty<std::string>("Coordinate Systems",
        std::bind(SELECT<const std::string&>::OVERLOAD_OF<INavigator>(&INavigator::setCoordinateSystems), nav, std::placeholders::_1),
        std::bind(&INavigator::stringifyCoordinateSystems, nav));

      declareProperty<double>("Position Threshold",
        std::bind(SELECT<const double>::OVERLOAD_OF<INavigator>(&INavigator::setPositionThreshold), nav, std::placeholders::_1),
        std::bind(&INavigator::positionThreshold, nav));

      declareProperty<double>("Orientation Threshold",
        std::bind(SELECT<const double>::OVERLOAD_OF<INavigator>(&INavigator::setOrientationThreshold), nav, std::placeholders::_1),
        std::bind(&INavigator::orientationThreshold, nav));
    }

    NavigatorProperty::NavigatorProperty(const NavigatorProperty & rhs)
      : gstd::DynamicProperty(rhs)
    { }

    NavigatorProperty::NavigatorProperty(NavigatorProperty&& rhs)
      : gstd::DynamicProperty(rhs)
    { }

    char * INavigator::NavigatorFeatureToString(const INavigator::NavigatorFeature & f)
    {
      switch (f)
      {
      case updateCoordinateSystem:
        return "F: Coordinate System";
        break;
      case updateRobotTransform:
        return "F: States";
        break;
      case updateState:
        return "F: State";
        break;
      case updateAdaptedPath:
        return "F: Adapted Path";
        break;
      case updateCurrentTargetOrientation:
        return "F: Current Target Orientation";
        break;
      case debugMessages:
        return "F: Debug";
        break;
      default:
        return "invalid";
      }
    }

    INavigator::NavigatorFeature INavigator::StringToNavigatorFeature(const std::string & str)
    {
      if (str == "F: Coordinate System")
        return updateCoordinateSystem;
      else if (str == "F: States")
        return updateRobotTransform;
      else if (str == "F: State")
        return updateState;
      else if (str == "F: Adapted Path")
        return updateAdaptedPath;
      else if (str == "F: Current Target Orientation")
        return updateCurrentTargetOrientation;
      else if (str == "F: Debug")
        return debugMessages;
      else
        return invalid;
    }

    INavigator::INavigator()
      : mState(Uninitialized)
      , mPathHelper(mpInterpolator)
      , mActivatedFeatures{
        updateRobotTransform,
        //updateCoordinateSystem => not active
        updateCurrentTargetOrientation,
        updateAdaptedPath, 
        updateState
      }
      , mPositionThreshold(0.1)
      , mOrientationThreshold(5)
    {
    }

    INavigator::~INavigator()
    {
      if (getSupervisor() != nullptr) 
        try { getSupervisor()->clearTimers(*this); }
        catch (...) {}
    }

    void INavigator::setPath(const MukPath& path)
    {
      mpInterpolator->setInput(path); 
      prepareInterpolator();
    }

    IInterpolator* INavigator::getInterpolator() const
    {
      return mpInterpolator.get();
    }
    /**
    * will swap the passed interpolator with the present interpolator
    */
    void INavigator::assumeInterpolator(std::unique_ptr<IInterpolator>& pInterpolator)
    {
      assumeInterpolatorNoPrepare(pInterpolator); // this swaps interpolators
      // pInterpolator  -> "old" interpolator
      // mpInterpolator -> "new" interpolator
      if (!mpInterpolator     // has Interpolator now
        && pInterpolator)     // had Interpolator before
        mpInterpolator->setInput(pInterpolator->getInput());
      prepareInterpolator();
    }

    /**
    * assume the Interpolator without preparing it.
    */
    void INavigator::assumeInterpolatorNoPrepare(std::unique_ptr<IInterpolator>& pInterpolator)
    {
      if (state() == enNavigatorState::Running)
        throw MUK_EXCEPTION("Trying to set the Interpolator while Running.", name());
      mInitialResolution = pInterpolator->getResolution();
      mpInterpolator.swap(pInterpolator);
    }

    /**
    * initialize the passed Calibrator
    * should throw an exception, if not successful (e.g. Calibration not supported)
    */
    void INavigator::initCalibrator(CalibrationStateMachine & c)
    {
      if (std::string(c.name()) == "CalibratorDummy") return;
      throw MUK_EXCEPTION_SIMPLE(
        (boost::format("The Navigator %s does not support Calibration.") % name()).str().c_str());
    }
    /**
    */
    MukState INavigator::interpolate(const double& t) const
    {
      if (!mpInterpolator)
        throw MUK_EXCEPTION("The Interpolator of the Navigator has not been set.", name());
      return mPathHelper.interpolate(t);
    }

    /**
    */
    double INavigator::findNearestPositionOnPath(const Vec3d & pos)
    {
      if (!mpInterpolator)
        throw MUK_EXCEPTION("The Interpolator of the Navigator has not been set.", name());
      return mPathHelper.findNearestPositionOnPath(pos);
    }

    /**
    */
    void INavigator::prepareInterpolator()
    {
      // make sure Interpolationtype is resolution
      if (mpInterpolator->getInterpolationType() != IInterpolator::EnInterpolationTypes::resolution)
        mpInterpolator->setInterpolationType(IInterpolator::EnInterpolationTypes::resolution);
      mpInterpolator->interpolate();
      mPathHelper.calculateDistances();
    }

    enNavigatorState INavigator::state() const
    {
      return mState;
    }

    void INavigator::setNavigatorFeature(const NavigatorFeature feature, const bool value)
    {
      if (value)
        mActivatedFeatures.insert(feature);
      else
      {
        auto it = mActivatedFeatures.find(feature);
        if(it != mActivatedFeatures.end())
          mActivatedFeatures.erase(it);
      }
    }

    bool INavigator::isFeatureActivated(const NavigatorFeature feature) const
    {
      auto it = mActivatedFeatures.find(feature);
      return it != mActivatedFeatures.end();
    }

    void INavigator::setCoordinateSystems(const CoordinateSystemList & list)
    {
      mCoordinateSystems = list;
    }

    void INavigator::setCoordinateSystems(const std::string &str)
    {
      mCoordinateSystems.clear();
      boost::split(mCoordinateSystems, str, [](const char& c) { return c == ';'; });
    }

    const INavigator::CoordinateSystemList & INavigator::getCoordinateSystems() const
    {
      return mCoordinateSystems;
    }

    std::string INavigator::stringifyCoordinateSystems() const
    {
      return boost::join(mCoordinateSystems, ";");
    }

    void INavigator::setPositionThreshold(const double threshold)
    {
      mPositionThreshold = threshold;

      // also make sure the interpolator resolution is updated
      fixMinimumResolution();
    }

    double INavigator::positionThreshold() const
    {
      return mPositionThreshold;
    }

    void INavigator::setOrientationThreshold(const double threshold)
    {
      mOrientationThreshold = threshold;
    }

    double INavigator::orientationThreshold() const
    {
      return mOrientationThreshold;
    }

    void INavigator::setInitialResolution(const double resolution)
    {
      mInitialResolution = resolution;
    }

    void INavigator::fixMinimumResolution()
    {
      // update the resolution, make sure it is always reasonably better than the position threshold
      double new_res = positionThreshold() / INavigator::POSITION_RESOLUTION_FACTOR;
      if (mpInterpolator && mpInterpolator->getResolution() > new_res)
        mpInterpolator->setResolution(new_res);
    }

    /**
    */
    void INavigator::setState(const enNavigatorState &newState)
    {
      if (newState == mState) return;
      mState = newState;
      if(isFeatureActivated(updateState))
        getSupervisor()->setNavigatorState(mState);
    }

    /**
    */
    INavigator::PathHelper::PathHelper(std::unique_ptr<IInterpolator>& pInterpolator) : mpInterpolator(pInterpolator)
    {}

    /** does not Interpolate
    */
    void INavigator::PathHelper::calculateDistances()
    {
      mPath = mpInterpolator->getInterpolation();
      mDistanceMap.clear();
      if (mPath.getStates().size() == 0)
        return;
      auto it = mPath.getStates().cbegin();
      const Vec3d* lastPosition = &it->coords;
      double t = 0.0;
      mDistanceMap.emplace(t, *it);
      ++it;
      for (; it != mPath.getStates().end(); ++it)
      {
        mDistanceMap.emplace_hint(
          mDistanceMap.end(), // hint
            t += (it->coords - *lastPosition).norm(), // t-position
            *it); // state (position and tangent)
        lastPosition = &it->coords;
      }
    }

    /**
    */
    const INavigator::PathHelper::DistanceMap INavigator::PathHelper::getDistanceMap() const
    {
      return mDistanceMap;
    }

    MukState INavigator::PathHelper::interpolate(const double & t) const
    {
      auto after = mDistanceMap.upper_bound(t);
      if (after == mDistanceMap.end()) return (--mDistanceMap.end())->second;
      if (after == mDistanceMap.begin()) return (mDistanceMap.begin())->second;
      auto before = after;
      --before;
      // linear interpolation between lower and after
      double m = (t - after->first) / (before->first - after->first);
      Vec3d  tangent = after->second.tangent + m * (before->second.tangent - after->second.tangent);
      tangent.normalize();
      Vec3d  coords = after->second.coords + m * (before->second.coords - after->second.coords);
      return MukState(coords, tangent);
    }

    double INavigator::PathHelper::findNearestPositionOnPath(const Vec3d & pos)
    {
      double t = 0;
      static_assert (std::numeric_limits<double>::has_infinity, "We require double to 'have infinity'.");
      double minDistance = std::numeric_limits<double>::infinity();
//      Vec3d closePoint(0, 0, 0);
      for (auto& i : mDistanceMap)
      {
        double newDistance = (i.second.coords - pos).squaredNorm();
        if (newDistance < minDistance)
        {
          t = i.first;
//          closePoint = i.second.coords;
          minDistance = newDistance;
        }
      }
      return t;
    }

    ProtectedNavigator::ProtectedNavigator(INavigator * const pNavigatorToProtect, std::mutex& mutex)
      : mpNavigator(pNavigatorToProtect), mMutex(mutex)
    { }

    /**
    * returns a copy of the Interpolator used by the Navigator
    */
    std::unique_ptr<IInterpolator> ProtectedNavigator::getInterpolator() const
    {
      std::lock_guard<std::mutex> lock(mMutex);
      return GetInterpolatorFactory().create(mpNavigator->getInterpolator());
    }

}
}