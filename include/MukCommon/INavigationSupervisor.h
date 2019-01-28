#pragma once

#include "muk_common_api.h"
#include "LoggerWrapper.h"
#include "MukException.h"
#include "TimerOwner.h"
#include "MukVector.h"

#include <chrono>

namespace gris
{
  namespace muk
  {
    class MukTransform;
    class MukState;
    class MukPath;

    enum enRunMode
    {
      Interval,
      Continuous
    };

    enum enNavigatorState
    {
      Uninitialized,
      Initialized,
      Paused,
      AwaitingReplanning,
      Running

    };

    enum enConsumerState
    {
      Invalid,
      Valid
    };

    class MUK_COMMON_API INavigationSupervisor
    {
    public:
      // RunMode administration
      virtual void        setRunMode(const enRunMode mode)  { mRunMode = mode; }
      enRunMode           getRunMode() const                { return mRunMode; }
    private:
      enRunMode           mRunMode;

    public:
      // interface to timed events
      // typedefs
      typedef int         TimerID;
      typedef std::chrono::milliseconds TimerInterval;
      typedef             std::function<void(const TimerID)> TimerFunction;
      typedef             std::function<void(void)>          DelayedFunction;
      // methods
      virtual TimerID     addTimer(const TimerOwner& owner, const TimerInterval interval, TimerFunction) = 0;
      virtual void        removeTimer(const TimerOwner& owner, const TimerID id) = 0;
      virtual void        clearTimers(const TimerOwner& owner) = 0;
      void                runDelayed(const TimerOwner& owner, const TimerInterval delay, DelayedFunction functor)
      { addTimer(owner, delay, [this, &owner, functor](TimerID id) { functor(); this->removeTimer(owner, id); }); }
    protected:
      virtual void        clearTimers() = 0;

    public:
      // interface to process outside events in Continuous Mode
      virtual void        processEvents(const int maxTime = 0) = 0;

    public:
      // add operations for information exchange here (avoids IObservable)
      virtual void        showMessageDialog(const std::string& msg, const bool WaitProceed) = 0;
      virtual void        showStatusMessage(const std::string& msg) = 0;
      virtual void        updateRobotTransform(const MukTransform& transform) = 0;
      virtual void        triggerReplanning(const MukState& currentPosition, const MukPath& path) = 0;
      virtual void        setNavigatorState(const enNavigatorState& newState) = 0;
      virtual void        setConsumerState(const enConsumerState& newState) = 0;
      virtual void        updateAdaptedPath(const MukPath& path) = 0;
      virtual void        updateCurrentTargetOrientation(const Vec3d& orientation) = 0;
      virtual void        updateCoordinateSystem(const MukTransform& transform) = 0;
      virtual void        forwardException(const MukException& exception) = 0;
      virtual void        forwardException(const std::exception& exception) = 0;

    public:
      // Logger interface
      virtual io::LoggerWrapper& Logger() = 0;
    };
  }
}


#define MUK_ASSERT_SUPERVISOR if(getSupervisor() == nullptr) throw MUK_EXCEPTION_SIMPLE("Supervisor of INavigator is not set")
#define LOG_NAV getSupervisor()->Logger() << name() << ": "
