#include "NavigationWorker.h"

#include "MukCommon/NavigatorFactory.h"
#include "MukCommon/InterpolatorFactory.h"
#include "MukCommon/CalibrationFactory.h"
#include "MukCommon/LoggerWrapper.h"

#include "gstd/InvokeLater.h"

#include <QAbstractEventDispatcher>
#include <QEventLoop>
#include <QTimerEvent>
#include <QThread>

#include <type_traits>

#include "boost/format.hpp"

// SL String Literal
#define SL(x) #x
#define MUK_NAV_ASSERT_NAVIGATOR \
if (!mpNavigator)\
  throw MUK_EXCEPTION_SIMPLE("Line " SL(__LINE__) ": No Navigator set.")
#define MUK_NAV_ASSERT_CALIBRATOR \
if (!mpCalibrator)\
  throw MUK_EXCEPTION_SIMPLE("Line " SL(__LINE__) ": No Calibrator set.")
#define MUK_NAV_ASSERT_INTERPOLATOR \
if (mpNavigator->getInterpolator() == nullptr)\
  throw MUK_EXCEPTION_SIMPLE("Line " SL(__LINE__) ": No Interpolator set.")
#define MUK_NAV_ASSERT_PATH \
if (mpNavigator->getInterpolator()->getInput().getPath().size() < 2)\
  throw MUK_EXCEPTION_SIMPLE("Line " SL(__LINE__ )": Path has less than 2 states.")

#define MUK_NAV_ASSERT_MEMBERS MUK_NAV_ASSERT_NAVIGATOR MUK_NAV_ASSERT_INTERPOLATOR MUK_NAV_ASSERT_PATH

#ifdef LOG_NAV
#undef LOG_NAV
#endif
#define LOG_NAV Logger() << "NavigationWorker: "


namespace gris
{
  namespace muk
  {

    const char* NavigationWorker::DEFAULT_NAVIGATOR = "NavigatorDummy";
    const char* NavigationWorker::DEFAULT_CALIBRATOR = "CalibratorDummy";

    NavigationWorker::NavigationWorker()
      : mTimerMutex()
      , mNavigatorMutex()
      , mCalibratorMutex()
      , mNavigatorRunMode(enRunMode::Interval)
      , mTerminate(false)
      , mPrivateSignals(this)
      , mpParentThread(thread())
      , mLogger([&](const std::string& str) { emit requestLogMessage(str); })
    {
      setNavigator(DEFAULT_NAVIGATOR);
      setCalibrator(DEFAULT_CALIBRATOR);
      MUK_NAV_ASSERT_NAVIGATOR;
      MUK_NAV_ASSERT_CALIBRATOR;
    }

    /**
    */
    void NavigationWorker::setRunMode(const enRunMode mode)
    {
      // clear Timers, if switching from Interval to Continuous
      if (getRunMode() != mode && mode == enRunMode::Continuous)
      {
        clearTimers();
      }
      INavigationSupervisor::setRunMode(mode);
    }

    /**
    */
    NavigationWorker::TimerID NavigationWorker::addTimer(const TimerOwner& owner, const NavigationWorker::TimerInterval interval, NavigationWorker::TimerFunction functor)
    {
      std::lock_guard<std::mutex> locker(mTimerMutex);
      TimerID id = startTimer(std::chrono::milliseconds(interval).count(), Qt::PreciseTimer);
      TimerIdentifierPair Identifier{ owner.getIdentifier(), id };
      mmTimers[Identifier] = functor;
      return id;
    }

    /**
    */
    void NavigationWorker::removeTimer(const TimerOwner& owner, const TimerID id)
    {
      std::lock_guard<std::mutex> locker(mTimerMutex);
      TimerIdentifierPair Identifier{ owner.getIdentifier(), id };
      auto it = mmTimers.find(Identifier);
      if (it != mmTimers.end()) removeTimerImpl(it);
    }

    /**
    */
    void NavigationWorker::clearTimers()
    {
      std::lock_guard<std::mutex> locker(mTimerMutex);
      for (auto it = mmTimers.begin(); it != mmTimers.end(); ++it)
         removeTimerImpl(it);
    }

    /**
    */
    void NavigationWorker::clearTimers(const TimerOwner& owner)
    {
      std::lock_guard<std::mutex> locker(mTimerMutex);
      for (auto it = mmTimers.begin(); it != mmTimers.end(); ++it)
        if (it->first.first == owner.getIdentifier()) removeTimerImpl(it);
    }

    /** maxTime in milliseconds
    */
    void NavigationWorker::processEvents(const int maxTime)
    {
      QEventLoop loop(this);
      const QEventLoop::ProcessEventsFlags loopFlags = QEventLoop::ExcludeUserInputEvents | QEventLoop::ExcludeSocketNotifiers;
      try
      {
        if (maxTime > 0)
          loop.processEvents(loopFlags, maxTime);
        else
          loop.processEvents(loopFlags);
      }
      catch (const std::exception& e)
      {
        Logger() << "Encountered Error in NavigationWorker.\n" << e.what() << std::endl;
        Logger().flush();
      }
    }

    /** returns a "shell" around the Navigator to ensure all operations are threadsafe
    *   Beware, this ProtectedNavigator Object expires and itself is only threadsafe, until
    *   the original Navigator behind it is changed. 
    */
    std::shared_ptr<ProtectedNavigator> NavigationWorker::makeProtectedNavigator()
    {
      std::lock_guard<std::mutex> lock(mNavigatorMutex);
      auto pObj = std::make_shared<QtProtectedNavigator>(this, mpNavigator.get(), mNavigatorMutex);
      return pObj;
    }

    /**
    */
    io::LoggerWrapper& NavigationWorker::Logger()
    {
      return mLogger;
    }

    /**
    */
    void NavigationWorker::removeTimerImpl(NavigationWorker::TimerMap::iterator& iterator)
    {
      killTimer(iterator->first.second);
      mmTimers.erase(iterator);
    }

    /**
    */
    void NavigationWorker::timerEvent(QTimerEvent * e)
    {
      std::unique_lock<std::mutex> locker(mTimerMutex);
      auto it = mmTimers.begin();
      while(it != mmTimers.end() && it->first.second != e->timerId())
        ++it;
      if (it == mmTimers.end())
      {
        std::string msg = (boost::format("TimerFunction (id: %d) could not be found") % e->timerId()).str();
        throw MUK_EXCEPTION_SIMPLE(msg.c_str())
      }
      else
      {
        TimerFunction f = it->second;
        locker.unlock();
        try {
          f(e->timerId());
        }
        catch (const MukException& e)
        {
          emit exceptionOccured(e);
        }
        catch (const std::exception& e)
        {
          emit exceptionOccured(e);
        }
      }
    }

    /**
    */
    void NavigationWorker::showMessageDialog(const std::string & msg, const bool waitProceed)
    {
      emit requestShowMessageDialog(msg, waitProceed);
    }

    /**
    */
    void NavigationWorker::showStatusMessage(const std::string & msg)
    {
      emit requestShowStatusMessage(msg);
    }

    /**
    */
    void NavigationWorker::updateRobotTransform(const MukTransform & transform)
    {
      emit robotTransformUpdated(transform);
    }

    void NavigationWorker::triggerReplanning(const MukState & currentPosition, const MukPath & path)
    {
      emit requestReplanning(currentPosition, path);
    }

    /**
    */
    void NavigationWorker::setNavigatorState(const enNavigatorState & newState)
    {
      emit navigatorStateUpdated(newState);
    }

    /**
    */
    void NavigationWorker::setConsumerState(const enConsumerState & newState)
    {
  //    if (mpNavigator && mpNavigator->state() == Running)
        emit consumerStateUpdated(newState);
    }

    /**
    */
    void NavigationWorker::updateAdaptedPath(const MukPath& path)
    {
      emit adaptedPathUpdated(path);
    }

    /**
    */
    void NavigationWorker::updateCurrentTargetOrientation(const Vec3d& orientation)
    {
      emit currentTargetOrientationUpdated(orientation);
    }

    void NavigationWorker::updateCoordinateSystem(const MukTransform & transform)
    {
      emit coordinateSystemUpdated(transform);
    }

    void NavigationWorker::forwardException(const gris::muk::MukException & e)
    {
      emit exceptionOccured(e);
    }

    void NavigationWorker::forwardException(const std::exception & e)
    {
      emit exceptionOccured(e);
    }

    /** is executed in the Navigator Thread, use with
    *     emit TabNavigation::navigatorChanged to trigger
    */
    void NavigationWorker::setNavigator(const std::string& newType)
    {
      enRunMode runmode = getRunMode();
      if (runmode == enRunMode::Continuous)
      {
        setRunMode(enRunMode::Interval);
        emit mPrivateSignals.postponeSetNavigator(newType);
      }
      else
        setNavigator_IntervalMode(newType);
    }

    void NavigationWorker::setCalibrator(const std::string & name)
    {
      MUK_NAV_ASSERT_NAVIGATOR;
      
      setCalibrator_IntervalMode(name);
    }

    /**
    * This slot should NEVER be triggered from outside the NavigationWorker class ;)
    */
    void NavigationWorker::setNavigator_IntervalMode(const std::string& newType)
    {
      try {
        if (getRunMode() == enRunMode::Continuous)
          throw MUK_EXCEPTION_SIMPLE("Trying to change the Navigator while in RunMode Continuous, possibly from within the Navigator. terminate() first and change the runmode. Also do NOT use postponeSetNavigator() directly.")
        std::unique_ptr<INavigator> pObj = GetNavigatorFactory().create(newType);
        // clearTimers stops all Navigator and Calibrator events
        clearTimers();
        {
          std::lock_guard<std::mutex> lock(mNavigatorMutex);
          mpNavigator.swap(pObj);
          mpNavigator->setSupervisor(this);
          emit navigatorChanged();
        }
        // only update if run in the Navigation thread
        if (mpParentThread != this->thread())
          updatePropertyReceiver();
        if (pObj)
        {
          pObj.release();
        }
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    void NavigationWorker::setCalibrator_IntervalMode(const std::string & newType)
    {
      try 
      {
        if (getRunMode() == enRunMode::Continuous)
        {
          throw MUK_EXCEPTION_SIMPLE("Trying to change the Calibrator while in RunMode Continuous, possibly from within the Navigator. terminate() first and change the runmode. Also do NOT use postponeSetNavigator() directly.")
        }
        auto pObj = GetCalibrationFactory().create(newType);
        clearTimers(*mpCalibrator);
        {
          std::lock_guard<std::mutex> lock(mCalibratorMutex);
          
          mpNavigator->initCalibrator(*pObj);

          LOG_NAV << "Calibrator: " << pObj->printableName() << " set." << std::endl; Logger().flush();

          mpCalibrator.swap(pObj);
          mpCalibrator->setSupervisor(this);
          emit calibratorChanged();
        }
        // only update if run in the Navigation thread
        if (mpParentThread != this->thread())
          updateCalibrationPropertyReceiver();
        if (pObj)
        {
          // this should "kill" all Navigator activity, i.e. also terminate
          pObj->terminate();
          pObj.release();
        }
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    enNavigatorState NavigationWorker::navigatorState() const
    {
      if (mpNavigator)
        return mpNavigator->state();
      else
        return Uninitialized;
    }

    enConsumerState NavigationWorker::consumerState() const
    {
      if (mpCalibrator)
        return mpCalibrator->state();
      else
        return Invalid;
    }

    bool NavigationWorker::isTerminated() const
    {
      return mTerminate;
    }

    void NavigationWorker::run()
    {
      // emit Navigator updated signal to show first Navigator
      emit navigatorChanged();
      updatePropertyReceiver();

      while (!mTerminate)
      {
        if (getRunMode() == enRunMode::Continuous)
        {
          bool reset;
          {
            std::lock_guard<std::mutex> lock(mNavigatorMutex);
            reset = mpNavigator->runContinuous();
          }
          if (reset)
          {
            setRunMode(enRunMode::Interval);
            clearTimers();
            setNavigator(mpNavigator->name());
          }
        }
        else
        {
          processEvents(1000);
        }
      }
      // kill all "Supervised" activities
      clearTimers();
      // this should "kill" all Navigator activity, i.e. also terminate
      if (mpNavigator && mpNavigator->state() != Uninitialized)
        mpNavigator->terminate();
      // this should kill all Calibrator activity, i.e. also terminate
      if (mpCalibrator && mpCalibrator->state() == Valid)
        mpCalibrator->terminate();
    }

    /**
    */
    void NavigationWorker::init(const std::string& interpolator, const StringVector& interpolatorProperties)
    {
      try {
        MUK_NAV_ASSERT_NAVIGATOR;
        std::lock_guard<std::mutex> lock(mNavigatorMutex);
        // using assumeInterpolatorNoPrepare() to improve performance (multiple Interpolation cycles)
        auto pInterpolator = GetInterpolatorFactory().create(interpolator);
        StringVector Properties;
        pInterpolator->getPropertyNames(Properties);
        if (Properties.size() != interpolatorProperties.size())
          throw MUK_EXCEPTION("Interpolator Properties inconsistent.", "Number of Properties differs with assumed values!");
        for (std::size_t i = 0; i < Properties.size(); ++i)
          pInterpolator->setProperty(Properties[i], interpolatorProperties[i]);

        mpNavigator->setInitialResolution(pInterpolator->getResolution());
        // old Interpolator will be swapped out
        mpNavigator->assumeInterpolatorNoPrepare(pInterpolator);
        mpNavigator->fixMinimumResolution();
        // if old Interpolator exists, set the associated path to the new Interpolator
        if (pInterpolator)
          mpNavigator->setPath(pInterpolator->getInput());
        mpNavigator->init();
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    void NavigationWorker::updatePath(const MukPath & path)
    {
      try {
        MUK_NAV_ASSERT_NAVIGATOR;
        std::lock_guard<std::mutex> lock(mNavigatorMutex);
        // using assumeInterpolatorNoPrepare() to improve performance (multiple Interpolation cycles)
        mpNavigator->setPath(path);
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    void NavigationWorker::calibrate()
    {
      try {
        MUK_NAV_ASSERT_CALIBRATOR
        if (mpCalibrator->state() != Valid)
            throw MUK_EXCEPTION_SIMPLE("Calibrator configuration is not valid");
        std::lock_guard<std::mutex> lock(mCalibratorMutex);
        mpCalibrator->init();
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    /**
    */
    void NavigationWorker::start()
    {
      try {
        MUK_NAV_ASSERT_NAVIGATOR
        if (mpNavigator->state() == Uninitialized)
          throw MUK_EXCEPTION_SIMPLE("Navigator not initialized");
        std::lock_guard<std::mutex> lock(mNavigatorMutex);
        mpNavigator->start();
//        setState(Running);
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    /**
    */
    void NavigationWorker::stop()
    {
      try {
        MUK_NAV_ASSERT_NAVIGATOR;
        std::lock_guard<std::mutex> lock(mNavigatorMutex);
        mpNavigator->stop();
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    /**
    */
    void NavigationWorker::proceed()
    {
      try {
        MUK_NAV_ASSERT_NAVIGATOR;
/*        if (!mpNavigator->isInitialized())
          throw MUK_EXCEPTION_SIMPLE("Navigator not initialized");*/
        {
          std::lock_guard<std::mutex> lock(mNavigatorMutex);

          mpNavigator->proceed();
        }
        if (mpCalibrator)
        {
          std::lock_guard<std::mutex> lock(mCalibratorMutex);
          if (mpCalibrator->state() == Valid && !mpCalibrator->isWaiting())
          mpCalibrator->proceed();
        }
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    /**
    */
    void NavigationWorker::terminate()
    {
      try {
        // not already terminated AND Navigator is valid
        if (!mTerminate && mpNavigator)
        {
          clearTimers();
          {
            std::lock_guard<std::mutex> lock(mNavigatorMutex);
            mpNavigator->terminate();
          }
          if (mpCalibrator)
          {
            std::lock_guard<std::mutex> lock(mCalibratorMutex);
            mpCalibrator->terminate();
          }
        }
        mTerminate = true;
      }
      catch (const MukException& e) { emit exceptionOccured(e); }
      catch (const std::exception& e) { emit exceptionOccured(e); }
    }

    /**
    * set up internal NavigationWorker Connections
    */
    void NavigationWorker::setupConnections()
    {
      // although this happens in the SAME thread, queue this signal, so we can do it in a different "environment" 
      // (i.e. in Interval Mode, so it is safe to delete the Navigator)
      connect(&mPrivateSignals, &NavigationWorkerPrivateSignals::postponeSetNavigator, this, &NavigationWorker::setNavigator_IntervalMode, Qt::QueuedConnection);
    }

    /** the PropertyReceiver mutex should be unlocked when calling this method.
    */
    void NavigationWorker::updatePropertyReceiver()
    {
      // mpNavPropertyReceiver is deleted through pPropertyReceiver.reset()
      if (mpNavPropertyReceiver)
      // if the program hangs here, the mutex is locked at a different part of the code...
        mpNavPropertyReceiver->moveGuardToObjectForDeletion();
      // create the Receiver Object, make it child of the Worker, so it will live in the Navigation Thread
      auto pPropertyReceiver = std::make_unique<SafePropertyReceiver>(mpNavigator->getProperty(), this);
      pPropertyReceiver->setLogger(&mLogger);
      // find a pointer to the Sender Object
      auto propSender = pPropertyReceiver->getSender();
      // move the Sender Object to the correct Thread
      moveToParentThread(propSender);
      // connect the Sender Object to the Receicer Object
      propSender->connectToReceiver(pPropertyReceiver.get());
      // put the Receiver into the correct memory location
      mpNavPropertyReceiver.swap(pPropertyReceiver);
      // emit the update message
      emit navigatorPropertyReset(propSender);
    }

    void NavigationWorker::updateCalibrationPropertyReceiver()
    {
      // Properties for calibrators not yet implemented
    }

    /**
    */
    void NavigationWorker::moveToParentThread(QObject * pobj)
    {
      pobj->moveToThread(mpParentThread);
    }

    QtProtectedNavigator::QtProtectedNavigator(NavigationWorker * pWorker, INavigator * const pNavigatorToProtect, std::mutex & mutex)
      : ProtectedNavigator(pNavigatorToProtect, mutex)
      , QObject()
      , mpWorker(pWorker)
    {
      // is created in the "Main" Thread
      // signal is automatically disconnected on destruction
      connect(pWorker, &NavigationWorker::navigatorChanged, this, &QtProtectedNavigator::updateNavigator);
    }

    QtProtectedNavigator::QtProtectedNavigator(const QtProtectedNavigator & pn)
      : ProtectedNavigator(pn)
      , QObject()
      , mpWorker(pn.mpWorker)
    {
      // signal is automatically disconnected on destruction
      connect(mpWorker, &NavigationWorker::navigatorChanged, this, &QtProtectedNavigator::updateNavigator);
    }

    void QtProtectedNavigator::updateNavigator()
    {
      setNavigator(mpWorker->makeProtectedNavigator().get());
    }

}
}
