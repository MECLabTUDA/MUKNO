#pragma once

#include "ThreadedController.h"

#include "MukCommon/MukException.h"
#include "MukCommon/INavigator.h"
#include "MukCommon/INavigationContainer.h"
#include "MukCommon/MukScene.h"

#include "MukQt/TabNavigation.h"

#include "NavigationWorker.h"
#include "SafeProperties.h"

#include <QThread>

#include <memory>

namespace gris
{
  namespace muk
  {
    class WorkerInterface : public QObject
    {
      Q_OBJECT

    public:
      WorkerInterface(QObject* p = nullptr) : QObject(p) {}

     signals:
      // signals to WorkerThread
      void      requestInit(const std::string&, const gris::muk::StringVector&);
      void      requestCalibration();
      void      requestStart();
      void      requestStop();
      void      requestProceed();
      void      requestSetNavigator(const std::string& name);
      void      requestSetCalibrator(const std::string& name);
      void      requestTerminate();

      void      requestUpdatePath(const gris::muk::MukPath& path);
    };

    class NavigationThread : public ThreadedController, public INavigationContainer
    {
      Q_OBJECT

      friend NavigationWorker;

    public:

    public:
      NavigationThread(QObject *parent = Q_NULLPTR);
      virtual ~NavigationThread();

    public:
      // QThread implementation
      // actual Thread execution (detached part)
      virtual void              run() Q_DECL_OVERRIDE;
      
    public:
      //typedef
      virtual void              initialize();
      void                      setupConnections();
    private:
      void                      setupConnections(TabNavigation* tab);

    public:
      // ThreadedModels implementation
      virtual void              setModels(AppModels* pModels);

    public:
      // INavigationContainer implementation
      // use ProtectedNavigator carefully, it will LOCK all Navigator activity
      virtual std::shared_ptr<ProtectedNavigator> getNavigator();
      SafePropertySender*                         getProperties();
      virtual enNavigatorState  navigatorState();
      virtual enConsumerState   consumerState();
      // the Thread has left its run-Function()
      virtual bool              isTerminated();
      void                      init();

    public slots:
      // Navigation state updates / changes
      void                      updateNavigatorState(const gris::muk::enNavigatorState& newState);
      void                      updateConsumerState(const gris::muk::enConsumerState& newState);
      // 3D Visualization updates
      void                      updateRobotTransform(const gris::muk::MukTransform& transform);
      void                      updateAdaptedPath(const gris::muk::MukPath& path);
      void                      updatedCurrentTargetOrientation(const gris::Vec3d& orientation);
      // debug info
      void                      updateCoordinateSystem(const gris::muk::MukTransform& transform);
      void                      changeFeature(const gris::muk::INavigator::NavigatorFeature& feature, const bool& state);

      void                      triggerReplanning(const gris::muk::MukState& position, const gris::muk::MukPath& path);

      void                      handleException(const gris::muk::MukException& e);
      void                      handleException(const std::exception& e);
    signals:
      // signals to GUI
      void                      requestShowMessageDialog(const std::string& msg, const bool waitProceed);
      void                      requestShowStatusMessage(const std::string& msg);
      void                      requestLogMessage(const std::string& msg);
      // all connections to this signal should be QueuedConnections
      void                      navigatorPropertyReset();
      void                      navigatorChanged();

    public slots:
      // move the current SafePropertySender Object back to the Navigationthread and 
      // 
      void                      updateNavigatorProperty(SafePropertySender* obj);

    private:
      void                      cachePlanningSetup();

    private:
      // creating thread, should also be the thread, that NavigationThread "lives in", i.e. result of thread()
      QThread*                  mpInitialThread;

      NavigationWorker          mWorker;

      std::shared_ptr<MukScene> mpScene;
      // created in Worker Thread, (-> parent none)
      SafePropertySender*       mpPropertySender;

      // created in Main Thread, parent NavigationThread Object
      WorkerInterface           mWorkerInterface;

      // cache for valid path + index
      std::string mCachedPathCollectionKey;
      int         mCachedIndex;
    };
  }
}