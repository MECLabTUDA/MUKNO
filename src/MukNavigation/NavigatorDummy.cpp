#include "private/muk.pch"
#include "NavigatorDummy.h"

#include "MukCommon/NavigatorFactory.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukTransform.h"
#include "MukCommon/muk_common.h"

#include "vtkSmartPointer.h"
#include "vtkTransform.h"

#include <chrono>
#include <thread>

namespace gris
{
  namespace muk
  {
    REGISTER_NAVIGATOR(NavigatorDummy);
    
    const std::chrono::milliseconds NavigatorDummy::UPDATE_INTERVAL{ 100 };
    const std::chrono::duration<double, std::milli> NavigatorDummy::SPEED_INV{ 1 / 0.001 }; // speed = 0.001 mm/millisecond = 1 mm/second

    /**
    */
    NavigatorDummyProperty::NavigatorDummyProperty(NavigatorDummy* nav)
      : NavigatorProperty(nav)
    {
      declareProperty<double>("Speed [mm/s]",
        std::bind(&NavigatorDummy::setSpeed, nav, std::placeholders::_1),
        std::bind(&NavigatorDummy::getSpeed, nav));
      declareProperty<long long>("UpdateInterval [ms]",
        std::bind(&NavigatorDummy::setUpdateInterval, nav, std::placeholders::_1),
        std::bind(&NavigatorDummy::getUpdateInterval, nav));
      declareProperty<std::string>("Export Filename",
        std::bind(&NavigatorDummy::setExportFilename, nav, std::placeholders::_1),
        std::bind(&NavigatorDummy::getExportFilename, nav));
    }

    /**
    */
    NavigatorDummy::NavigatorDummy()
      : INavigator()
      , mSpeed_inv(SPEED_INV)
      , mExportFilename("")
      , mUpdateInterval(UPDATE_INTERVAL)
      , mStartTime(std::chrono::steady_clock::now())
      , mPauseTime(std::chrono::steady_clock::time_point::min())
      , mOffset(std::chrono::nanoseconds::zero())
      , mReplanningStateBuffer()
    {
    }

    /**
    */
    void NavigatorDummy::init()
    {
      MUK_ASSERT_SUPERVISOR;
      if (getInterpolator() == nullptr)
        throw MUK_EXCEPTION("Interpolator not set!", name());
      LOG_NAV << "initialized" << std::endl; getSupervisor()->Logger().flush();
      setState(Initialized);
    }

    /**
    */
    void NavigatorDummy::start()
    {
      mStartTime = std::chrono::steady_clock::now();
      mPauseTime = mPauseTime.min();
      mOffset = std::chrono::nanoseconds::zero();
      // if not initialized, fail
      if (Uninitialized)
        throw MUK_EXCEPTION("Cannot start Navigator!", "Navigator is not initialized.");
      proceedImpl();
      LOG_NAV << "started" << std::endl; getSupervisor()->Logger().flush();
    }

    /**
    */
    void NavigatorDummy::tick(INavigationSupervisor::TimerID tid)
    {
      bool bupState = isFeatureActivated(updateRobotTransform);
      bool bexport  = !mExportFilename.empty();
      bool bupCurrentTargetOrientation = isFeatureActivated(updateCurrentTargetOrientation);

      MukState current = currentState();
      
      const auto& path_vec = getInterpolator()->getInput().getStates();
      if (path_vec.size() > 1
        && (current.coords - (*(path_vec.end() - 1)).coords).squaredNorm() < 1.5*positionThreshold())
      {
        getSupervisor()->showMessageDialog(std::string(name()) + " reached a position close to the goal, Finished!", false);
        stop();
      }
      else if (state() != AwaitingReplanning)
      {
        auto nearest = interpolate(findNearestPositionOnPath(current.coords));
        auto distToPath = (current.coords - nearest.coords);
        auto angleToPath = acos(current.tangent.normalized().dot(nearest.tangent.normalized())) / atan(1) * 90;

        if (isFeatureActivated(debugMessages))
        {
          LOG_NAV << "\nDistance to Path: " << distToPath << "\nAngle to Path: " << angleToPath << std::endl;
          getSupervisor()->Logger().flush();
        }

        // evaluates threshold for replanning
        if (distToPath.squaredNorm() > positionThreshold()
          || angleToPath > orientationThreshold())
        {
          getSupervisor()->showStatusMessage("Threshold for Navigation exceeded, waiting for replanning.");
          mReplanningStateBuffer = current;
          getSupervisor()->triggerReplanning(current, getInterpolator()->getInput());
          haltMovement();
          setState(AwaitingReplanning);
        }
      }
      else
      {
        if ((std::chrono::steady_clock::now() - mPauseTime).count() > 30000000) // nanoseconds -> 30 s
          getSupervisor()->showStatusMessage("Still waiting for replanning...");
      }

      if (!bupState && !bupCurrentTargetOrientation) return;

      if (bupState || bexport)
      {
        auto pTransform = vtkSmartPointer<vtkTransform>::New();
        auto pMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
        pMatrix->Identity();
        // set the position in the transform
        pMatrix->SetElement(0, 3, current.coords.x());
        pMatrix->SetElement(1, 3, current.coords.y());
        pMatrix->SetElement(2, 3, current.coords.z());
        // set the normal orientation in the transform
        current.tangent.normalize();
        pMatrix->SetElement(0, 0, current.tangent.x());
        pMatrix->SetElement(1, 0, current.tangent.y());
        pMatrix->SetElement(2, 0, current.tangent.z());

        // set the y direction as cross product of x direction and current position
        Vec3d yDirection = current.tangent.cross(current.coords);
        yDirection.normalize();
        pMatrix->SetElement(0, 1, yDirection.x());
        pMatrix->SetElement(1, 1, yDirection.y());
        pMatrix->SetElement(2, 1, yDirection.z());

        // set the z direction as cross product of x and y direction
        Vec3d zDirection = yDirection.cross(current.tangent);
        zDirection.normalize();
        pMatrix->SetElement(0, 2, zDirection.x());
        pMatrix->SetElement(1, 2, zDirection.y());
        pMatrix->SetElement(2, 2, zDirection.z());

        // set the Transform of the Robot
        pTransform->SetMatrix(pMatrix);
        MukTransform transform(pTransform, "Robot");
        if (bupState)
          getSupervisor()->updateRobotTransform(transform);
        if (bexport)
          pMatrix->Print(mExportStream);
      }
      if(bupCurrentTargetOrientation) getSupervisor()->updateCurrentTargetOrientation(current.tangent );
    }

    /**
    */
    void NavigatorDummy::adaptPath(INavigationSupervisor::TimerID)
    {
      if (isFeatureActivated(updateAdaptedPath))
      {
        const size_t NUM_TARGETPATH = 10;
        std::chrono::nanoseconds t = (std::chrono::steady_clock::now() - mStartTime) - mOffset;
        auto d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t) / mSpeed_inv;
        MukPath adaptedPath = getPath();
        std::vector<MukState> adaptedPathStates;
        adaptedPathStates.reserve(NUM_TARGETPATH);
        for (size_t i = 0; i < NUM_TARGETPATH; ++i)
        {
          adaptedPathStates.push_back(interpolate(d + std::chrono::duration<double, std::milli>(2000 * i / NUM_TARGETPATH) / mSpeed_inv));
        }
        adaptedPath.setStates(adaptedPathStates);
        getSupervisor()->updateAdaptedPath(adaptedPath);
      }
    }

    /**
    */
    void NavigatorDummy::stop()
    {
      haltMovement();
      if (mExportStream.is_open()) mExportStream.close();
      getSupervisor()->clearTimers(*this);
      setState(Initialized);
      LOG_NAV << "stopped" << std::endl; getSupervisor()->Logger().flush();
    }

    /**
    */
    void NavigatorDummy::proceed()
    {
      if (state() == AwaitingReplanning)
        postReplanning();

      if (mPauseTime < mStartTime)
        mOffset = mOffset.zero();
      else
        mOffset += std::chrono::steady_clock::now() - mPauseTime;
      proceedImpl();
      LOG_NAV << "continued" << std::endl; getSupervisor()->Logger().flush();
    }

    /**
    */
    void NavigatorDummy::proceedImpl()
    {
      getSupervisor()->addTimer(*this, mUpdateInterval, 
        std::bind(&NavigatorDummy::tick, this, std::placeholders::_1));
      if(!mExportFilename.empty()) openfile();
      setState(Running);
    }

    void NavigatorDummy::haltMovement()
    {
      mPauseTime = std::chrono::steady_clock::now();
    }

    /**
    */
    bool NavigatorDummy::runContinuous()
    {
      //try {
      // setRunning(true);

      // do nothing, wrong mode, switch to Interval Mode
      getSupervisor()->setRunMode(enRunMode::Interval);

      // process events of the "Main Program" (this calls start(), stop(), etc.)
      // getSupervisor()->processEvents();
      // if terminate has been called, return false and leave runContinuous!
      // if (mTerminateFlag) return false;

      // } catch (...) {}
      // setRunning(false);
      return false;
    }

    /**
    */
    void NavigatorDummy::terminate()
    {
      //mTerminateFlag = true;
      stop();
      setState(Uninitialized);
    }

    void NavigatorDummy::preReplanning()
    {
    }

    void NavigatorDummy::postReplanning()
    {
    }

    MukState NavigatorDummy::currentState()
    {
      std::chrono::nanoseconds t;
      if (state() == AwaitingReplanning)
        t = (mPauseTime - mStartTime) - mOffset;
      else if (state() == Running)
        t = (std::chrono::steady_clock::now() - mStartTime) - mOffset;
      else
        throw MUK_EXCEPTION_SIMPLE("Navigator State neither in Running or Awaiting Replanning, but currentState() called.");

      if (isFeatureActivated(debugMessages))
      {
        LOG_NAV << "state: " << (t.count() / 10e6) << std::endl;
        getSupervisor()->Logger().flush();
      }
      auto d = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t) / mSpeed_inv;
      return interpolate(d);
    }

    std::unique_ptr<NavigatorProperty> NavigatorDummy::getProperty()
    {
      return std::make_unique<NavigatorDummyProperty>(this);
    }

    void NavigatorDummy::setSpeed(const double speed)
    {
      if (speed == 0) mSpeed_inv = seconds(0);
      else mSpeed_inv = seconds(1 / speed);
    }

    double NavigatorDummy::getSpeed() const
    {
      if (mSpeed_inv == decltype(mSpeed_inv)::zero()) return 0;
      else return 1 / seconds(mSpeed_inv).count();
    }
    void NavigatorDummy::setPath(const MukPath & path)
    {
      INavigator::setPath(path);

      auto old = mStartTime;
      // path is set, reset to initial position on path
      mStartTime = std::chrono::steady_clock::now();
      mOffset    = std::chrono::nanoseconds::zero();

      if (isFeatureActivated(debugMessages))
      {
        LOG_NAV << "delta: " << (mStartTime - old).count() / 10e6 << std::endl;
        getSupervisor()->Logger().flush();
      }
    }
    void NavigatorDummy::setUpdateInterval(const std::chrono::milliseconds::rep interval)
    {
      mUpdateInterval = std::chrono::milliseconds(interval);
    }
    std::chrono::milliseconds::rep NavigatorDummy::getUpdateInterval() const
    {
      return mUpdateInterval.count();
    }
    void NavigatorDummy::setExportFilename(const std::string & filename)
    {
      mExportFilename = filename;
      if (state() == Running) openfile();
    }
    std::string NavigatorDummy::getExportFilename() const
    {
      return mExportFilename;
    }
    void NavigatorDummy::openfile()
    {
      if (mExportStream.is_open()) mExportStream.close();
      if (!mExportFilename.empty()) // add check folder exists?
        mExportStream.open(mExportFilename, std::ios_base::out | std::ios_base::ate);
    }
  }
}