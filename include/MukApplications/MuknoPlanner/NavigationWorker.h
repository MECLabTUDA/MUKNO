#pragma once

#include "private/muk.pch"
#include "MukAppModels/VisualizationModel.h"
#include "SafeProperties.h"

#include "MukCommon/INavigationSupervisor.h"

#include "MukCommon/Typedefs.h"
#include "MukCommon/MukException.h"
#include "MukCommon/INavigator.h"
#include "MukCommon/IInterpolator.h"
#include "MukCommon/MukPath.h"
#include "MukCommon/MukVector.h"
#include "MukCommon/LoggerWrapper.h"

#include <QObject>

namespace gris
{
  namespace muk
  {
    class NavigationThread;
    class QtProtectedNavigator;

    /**
    */
    class NavigationWorkerPrivateSignals : public QObject 
    {
      Q_OBJECT

      public:
        NavigationWorkerPrivateSignals(QObject* p = nullptr) : QObject(p) {}
      signals:
        // internal (private) signals
        void                      postponeSetNavigator(const std::string& name);
        void                      requestTerminate();
    };

    /**
    */
    class NavigationWorker 
      : public QObject
      , public INavigationSupervisor 
    {

      Q_OBJECT

      public:
        static const char* DEFAULT_NAVIGATOR;
        static const char* DEFAULT_CALIBRATOR;

      private:
        typedef std::pair<std::string, TimerID> TimerIdentifierPair;
        typedef std::map<TimerIdentifierPair, TimerFunction> TimerMap;

      public:
        NavigationWorker();
        NavigationWorker(NavigationWorker&)  = delete;
        NavigationWorker(NavigationWorker&&) = delete;

        NavigationWorker& operator=(NavigationWorker&)  = delete;
        NavigationWorker& operator=(NavigationWorker&&) = delete;

      public:

        // INavigationSupervisor implementations
        // RunMode administration
        virtual void              setRunMode(const enRunMode mode);
        // interface to timed events
        virtual TimerID           addTimer(const TimerOwner& owner, const TimerInterval interval, TimerFunction functor);
        virtual void              removeTimer(const TimerOwner& owner, const TimerID id);
        virtual void              clearTimers(const TimerOwner& owner);
        // interface to process outside events in Continuous Mode
        virtual void              processEvents(const int maxTime = 0);
    protected:
        virtual void              clearTimers();

      private:
        void                      removeTimerImpl(TimerMap::iterator & iterator);
      protected:
        virtual void              timerEvent(QTimerEvent* e);

      public:
        virtual void              showMessageDialog(const std::string& msg, const bool waitProceed);
        virtual void              showStatusMessage(const std::string& msg);
        virtual void              updateRobotTransform(const MukTransform& state);
        virtual void              triggerReplanning(const MukState& currentPosition, const MukPath& path);
        virtual void              setNavigatorState(const enNavigatorState& newState);
        virtual void              setConsumerState(const enConsumerState& newState);
        virtual void              updateAdaptedPath(const MukPath& path);
        virtual void              updateCurrentTargetOrientation(const Vec3d& orientation);
        virtual void              updateCoordinateSystem(const MukTransform& transform);
        virtual void              forwardException(const gris::muk::MukException& e);
        virtual void              forwardException(const std::exception& e);

      signals:
        // signals for the Thread-Object to listen to
        void                      requestShowMessageDialog(const std::string& msg, const bool waitProceed);
        void                      requestShowStatusMessage(const std::string& msg);
        void                      requestLogMessage(const std::string& msg);
        void                      robotTransformUpdated(const gris::muk::MukTransform& transform);
        void                      requestReplanning(const gris::muk::MukState& position, const gris::muk::MukPath& path);
        void                      navigatorStateUpdated(const gris::muk::enNavigatorState& newState);
        void                      consumerStateUpdated(const gris::muk::enConsumerState& newState);
        void                      adaptedPathUpdated(const gris::muk::MukPath& path);
        void                      currentTargetOrientationUpdated(const gris::Vec3d& orientation);
        void                      coordinateSystemUpdated(const gris::muk::MukTransform& transform);
        void                      exceptionOccured(const gris::muk::MukException& e);
        void                      exceptionOccured(const std::exception& e);
        void                      navigatorChanged();
        void                      navigatorPropertyReset(SafePropertySender* sender);
        void                      calibratorChanged();
        void                      calibratorPropertyReset(SafePropertySender* sender);

      public slots:
        // slots for Crossthread communication
        void                      init(const std::string& interpolator, const StringVector& interpolatorProperties);
        void                      updatePath(const MukPath& path);
        void                      calibrate();
        void                      start();
        void                      stop();
        void                      proceed();
        void                      setNavigator(const std::string& name);
        void                      setCalibrator(const std::string& name);
        void                      terminate();
      private slots:
        // internal slot
        void                      setNavigator_IntervalMode(const std::string& name);
        void                      setCalibrator_IntervalMode(const std::string& name);

      public:
        enConsumerState           consumerState() const;
        enNavigatorState          navigatorState() const;
        // the Thread has left its run-Function()
        bool                      isTerminated() const;
        std::shared_ptr<ProtectedNavigator>        makeProtectedNavigator();

      public:
        // Logger interface
        virtual io::LoggerWrapper& Logger();

      private:
        void                      setupConnections();
        void                      updatePropertyReceiver();
        void                      updateCalibrationPropertyReceiver();
        void                      moveToParentThread(QObject* pobj);

      public:
        void                      run();

      private:
        std::unique_ptr<INavigator>              mpNavigator;
        std::unique_ptr<CalibrationStateMachine> mpCalibrator;
        std::unique_ptr<SafePropertyReceiver>    mpNavPropertyReceiver;
        std::unique_ptr<SafePropertyReceiver>    mpCalibPropertyReceiver;
        TimerMap                                 mmTimers;
        enRunMode                                mNavigatorRunMode;
        std::mutex                               mTimerMutex;
        std::mutex                               mNavigatorMutex;
        std::mutex                               mCalibratorMutex;

        // mPrivateSignals is created with the worker as parent and therefore
        // "lives" in the Navigation Thread (i.e. moved to the Navigation Thread)
        NavigationWorkerPrivateSignals        mPrivateSignals;
        std::atomic_bool                      mTerminate;

        io::LoggerWrapper                     mLogger;

        QThread*                              mpParentThread;
    };

    class QtProtectedNavigator : public QObject, public ProtectedNavigator
    {

      Q_OBJECT

    public:
      QtProtectedNavigator(NavigationWorker* parent, INavigator* const pNavigatorToProtect, std::mutex& mutex);
      QtProtectedNavigator(const QtProtectedNavigator& pn);

    public slots:
      void updateNavigator();

    private:
      NavigationWorker* mpWorker;
    };
  }
}
