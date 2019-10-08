#include "private/muk.pch"

#include "AppControllers.h"
#include "InteractionController.h"
#include "PlanningController.h"
#include "ProblemDefinitionController.h"
#include "PropertyController.h"
#include "SafeProperties.h"
#include "VisualizationController.h"

#include "NavigationThread.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"

#include "MukCommon/InterpolatorFactory.h"
#include "MukCommon/MukException.h"
#include "MukCommon/PathCollection.h"

#include "MukQt/muk_qt_tools.h"
#include "MukQt/MuknoPlannerMainWindow.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisPathCollection.h"

#include <boost/format.hpp>

#ifdef LOG_NAV
#undef LOG_NAV
#endif // LOG_NAV
#define LOG_NAV LOG_LINE << "NavigationThread: "

namespace
{
  const std::string postFix = "_replanning";
}

namespace gris
{
  namespace muk
  {

    /**
    */
    NavigationThread::NavigationThread(QObject *parent)
      : ThreadedController(parent)
      , mWorkerInterface(this)
      , mWorker() // no parent
      , mpPropertySender(nullptr)
      , mpScene(nullptr)
      , mpInitialThread(thread())
    {
      mWorker.moveToThread(this);
    }

    /**
    */
    NavigationThread::~NavigationThread()
    {
      emit mWorkerInterface.requestTerminate();
      int i = 0;
      // wait for the thread to finish or 1 second
      while (! this->wait(1000))
      { 
        if (isFinished()) break;
        ++i;
        LOG_LINE << "Waiting for NavigationThread to finish (currently waiting for at least " << i << " seconds).";
        emit mWorkerInterface.requestTerminate();
      }
    }

    /**
    */
    void NavigationThread::run()
    {
      mWorker.Logger() << "Now running NavigationThread..."  << std::endl;
      mWorker.Logger().flush();
      mWorker.run();
    }

    /**
    */
    void NavigationThread::setupConnections(TabNavigation* tab)
    {
      connect(mpMainWindow->mpTabContainer, &QTabWidget::currentChanged, this, &NavigationThread::cachePlanningSetup);
      // Messages from "Navigation thread"
      connect(this, &NavigationThread::requestShowMessageDialog,        tab, &TabNavigation::showMessageDialog);
      connect(this, &NavigationThread::requestShowStatusMessage,        tab, &TabNavigation::showStatusMessage);
      connect(this, &NavigationThread::requestLogMessage,               tab, &TabNavigation::logMessage);

      connect(tab, &TabNavigation::initRequested,     this, &NavigationThread::init);
      // forwarding some requests to the WorkerInterface
      connect(tab, &TabNavigation::navigatorChanged,  &mWorkerInterface, &WorkerInterface::requestSetNavigator);
      connect(tab, &TabNavigation::calibratorChanged, &mWorkerInterface, &WorkerInterface::requestSetCalibrator);
      connect(tab, &TabNavigation::proceedRequested,  &mWorkerInterface, &WorkerInterface::requestProceed);
      connect(tab, &TabNavigation::stopRequested,     &mWorkerInterface, &WorkerInterface::requestStop);
      connect(tab, &TabNavigation::startRequested,    &mWorkerInterface, &WorkerInterface::requestStart);
      connect(tab, &TabNavigation::calibrationRequested, &mWorkerInterface, &WorkerInterface::requestCalibration);
    }

    /**
    */
    void NavigationThread::setupConnections()
    {
      // listening to changes of the pathCollection does not work :(

      // these should be automatically Queued Connections
      connect(&mWorker, &NavigationWorker::requestShowMessageDialog,        this, &NavigationThread::requestShowMessageDialog);
      connect(&mWorker, &NavigationWorker::requestShowStatusMessage,        this, &NavigationThread::requestShowStatusMessage);
      connect(&mWorker, &NavigationWorker::requestLogMessage,               this, &NavigationThread::requestLogMessage);
      connect(&mWorker, &NavigationWorker::robotTransformUpdated,           this, &NavigationThread::updateRobotTransform);
      connect(&mWorker, &NavigationWorker::navigatorStateUpdated,           this, &NavigationThread::updateNavigatorState);
      connect(&mWorker, &NavigationWorker::consumerStateUpdated,            this, &NavigationThread::updateConsumerState);
      connect(&mWorker, &NavigationWorker::requestReplanning, this, &NavigationThread::triggerReplanning);
      connect(&mWorker, &NavigationWorker::adaptedPathUpdated,              this, &NavigationThread::updateAdaptedPath);
      connect(&mWorker, &NavigationWorker::currentTargetOrientationUpdated, this, &NavigationThread::updatedCurrentTargetOrientation);
      connect(&mWorker, &NavigationWorker::coordinateSystemUpdated,         this, &NavigationThread::updateCoordinateSystem);
      connect(&mWorker, SELECT<const MukException&>::OVERLOAD_OF<NavigationWorker>(&NavigationWorker::exceptionOccured), this, SELECT<const MukException&>::OVERLOAD_OF<NavigationThread>(&NavigationThread::handleException));
      connect(&mWorker, SELECT<const std::exception&>::OVERLOAD_OF<NavigationWorker>(&NavigationWorker::exceptionOccured), this, SELECT<const std::exception&>::OVERLOAD_OF<NavigationThread>(&NavigationThread::handleException));
      connect(&mWorker, &NavigationWorker::navigatorChanged,                this, &NavigationThread::navigatorChanged);
      // this also Blocks (i.e. waits for completion in order to synchronize)
      connect(&mWorker, &NavigationWorker::navigatorPropertyReset,          this, &NavigationThread::updateNavigatorProperty, Qt::BlockingQueuedConnection);
      

      // WorkerInterface is in Main Thread, Worker is in NavigationThread --> automatically Queued Connections
      connect(&mWorkerInterface, &WorkerInterface::requestInit,             &mWorker, &NavigationWorker::init);
      connect(&mWorkerInterface, &WorkerInterface::requestUpdatePath,       &mWorker, &NavigationWorker::updatePath);
      connect(&mWorkerInterface, &WorkerInterface::requestStart,            &mWorker, &NavigationWorker::start);
      connect(&mWorkerInterface, &WorkerInterface::requestStop,             &mWorker, &NavigationWorker::stop);
      connect(&mWorkerInterface, &WorkerInterface::requestProceed,          &mWorker, &NavigationWorker::proceed);
      connect(&mWorkerInterface, &WorkerInterface::requestSetNavigator,     &mWorker, &NavigationWorker::setNavigator);
      connect(&mWorkerInterface, &WorkerInterface::requestSetCalibrator,    &mWorker, &NavigationWorker::setCalibrator);
      connect(&mWorkerInterface, &WorkerInterface::requestTerminate,        &mWorker, &NavigationWorker::terminate);

      //connect(this, &NavigationThread::adaptedPathUpdated,                  [this](const MukPath& p)  {LOG_LINE << "adaptedPath: " << p.getStates()[0] << " : " << p.getStates()[1]; });
      //connect(this, &NavigationThread::currentTargetOrientationUpdated,     [this](const Vec3d& p)    {LOG_LINE << "orientation: " << p; });
      //connect(this, &NavigationThread::stateUpdated,                        [this](const MukState& p) {LOG_LINE << "currentstate: " << p; });
      auto pTab = mpMainWindow->mpTabNavigation;
      setupConnections(pTab);
    }

    /**
    */
    void NavigationThread::initialize()
    {
      mpScene = mpModels->pAppModel->getScene();
      mpMainWindow->mpTabNavigation->initialize(mpScene);
    }

    /**
    */
    void NavigationThread::cachePlanningSetup()
    {
      auto key = mpModels->pPlanningModel->getActivePathCollection();
      if (key.find(postFix) != std::string::npos)
      {
        // we switched during navigation
        key.resize( key.size() - postFix.size() );
      }
      else
      {
        // we switched while no navigation was offline
        mCachedIndex = mpModels->pPlanningModel->getActivePathIdx();
      }
      mCachedPathCollectionKey = key;
      mpControls->mpInteract->setDefaultInteraction();
    }

    /**
    * read data from "Main Thread" and forward to Worker::init
    */
    void NavigationThread::init()
    {
      StringVector values;
      mpScene->getInterpolator()->getPropertyValues(values);
      emit mWorkerInterface.requestInit(mpScene->getInterpolator()->name(), values);

      LOG_NAV << "reading collection " << mCachedPathCollectionKey;
      const auto& pathCollection = mpScene->getPathCollection(mCachedPathCollectionKey);
      if (mCachedIndex == PlanningModel::Invalid_Index || mCachedIndex >= (int)pathCollection.getPaths().size())
      {
        std::string detail = "current Pathcollection: " + mCachedPathCollectionKey + "; current path number: " + std::to_string(mCachedIndex+1);
        std::string msg = (boost::format("A path with number %d (internal index: %d) is not available.") % (mCachedIndex + 1) % mCachedIndex).str();
        throw MUK_EXCEPTION(msg.c_str(), detail.c_str());
      }
      if (pathCollection.getPaths().empty())
      {
        std::string detail = "current Pathcollection: " + mCachedPathCollectionKey + "; current path number: " + std::to_string(mCachedIndex+1);
        std::string msg = (boost::format("No paths available! Passed index %d (internal index: %d).") % (mCachedIndex + 1) % mCachedIndex).str();
        throw MUK_EXCEPTION(msg.c_str(), detail.c_str());
      }

      const MukPath& path = pathCollection.getPaths()[mCachedIndex];
      if(path.getStates().size() < 2)
      {
        throw MUK_EXCEPTION_SIMPLE("Trying to initialize Navigator, but the current Path has less than 2 Points.");
      }

      // create the path collection that is only used for replanning this path
      const auto name = mCachedPathCollectionKey + postFix;
      // start from scratch
      {
        if (mpScene->hasPathKey(name))
          mpControls->mpPlanningController->deletePathCollection(name);
        mpControls->mpPlanningController->addPathCollection(name);
      }
      // then setup the new one
      {
        // copy the reference problem definition to the new one
        auto& coll = mpScene->getPathCollection(name);
        coll.getProblemDefinition()->copyParameters(*pathCollection.getProblemDefinition());
        for (const auto& key : pathCollection.getInactiveObstacles())
          coll.addObstacle(key);
        // specify specific start and goal regions
        auto start = std::make_unique<MukStateRegion>();
        start->setCenter( path.getStates().front() );
        start->setPhi(0.0);
        start->setRadius(0.0);
        start->setResolution(0);
        mpControls->mpProbDefControl->addStartRegion(std::move(start));
        auto goal = std::make_unique<MukStateRegion>();
        goal->setCenter( path.getStates().back() );
        goal->setPhi(0.0);
        goal->setRadius(0.0);
        goal->setResolution(0);
        mpControls->mpProbDefControl->addGoalRegion(std::move(goal));
        mpControls->mpPlanningController->updateProblemDefinition(name);
        mpControls->mpPropControl->initSceneWidget();
      }
      emit mWorkerInterface.requestUpdatePath(path);
    }

    /**
    */
    void NavigationThread::setModels(AppModels* pModels)
    {
      pModels->pAppModel->getScene()->setNavigation(this);
      ThreadedController::setModels(pModels);
    }

    std::shared_ptr<ProtectedNavigator> NavigationThread::getNavigator()
    {
      return mWorker.makeProtectedNavigator();
    }

    SafePropertySender* NavigationThread::getProperties()
    {
      return mpPropertySender;
    }

    enNavigatorState NavigationThread::navigatorState()
    {
      return mWorker.navigatorState();
    }

    enConsumerState NavigationThread::consumerState()
    {
      return mWorker.consumerState();
    }

    bool NavigationThread::isTerminated()
    {
      return mWorker.isTerminated();
    }

    // this should be run in a BlockingQueuedConnection, i.e. the Navigator Thread waits for its 
    // completion
    void NavigationThread::updateNavigatorProperty(SafePropertySender* sender)
    {
      // move the old sender back to the NavigationThread, so it can be destroyed there
      // by the corresponding SafeProperty
      if (mpPropertySender != nullptr)
        mpPropertySender->moveToThread(this);
      mpPropertySender = sender;
      emit navigatorPropertyReset();
    }
    
    /**
    */
    void NavigationThread::updateNavigatorState(const gris::muk::enNavigatorState& newState)
    {
      mpModels->pVisModel->updateNavState_Initialization(newState != Uninitialized);
      mpMainWindow->mpTabNavigation->toggleActivationInitialization(newState != Uninitialized);
      mpControls->mpPropControl->initSceneWidget();
      mpControls->mpVisControl->updateNavState_Running(newState == Running || newState == AwaitingReplanning);
      mpMainWindow->mpTabNavigation->toggleActivationRunning(newState == Running || newState == AwaitingReplanning);
    }

    /**
    */
    void NavigationThread::updateConsumerState(const enConsumerState& newState)
    {
      //mpModels->pVisModel->updateNavState_Running(newState);
      mpMainWindow->mpTabNavigation->toggleActivationCalibratorValid(newState == Valid);
    }

    /**
    */
    void NavigationThread::changeFeature(const gris::muk::INavigator::NavigatorFeature& feature, const bool& state)
    {
      mpModels->pVisModel->updateNavigatorFeature(feature, state);
    }

    /** \brief performs replanning if misalignment/distance/other threshold in #INavigationSupervisor is violated

      \param position: current position
      \param path: current path
    */
    void NavigationThread::triggerReplanning(const MukState & position, const MukPath & path)
    {
      // create the path collection that is only used for replanning this path
      const auto name = mpModels->pPlanningModel->getActivePathCollection();
      LOG_NAV << "replanning for '" << name << "'";
      
      auto& coll = mpScene->getPathCollection(name);
      {
        mpControls->mpProbDefControl->deleteRegion(MukProblemDefinition::enStart, 0);
        auto start = std::make_unique<MukStateRegion>();
        start->setCenter( position );
        start->setPhi(0.0);
        start->setRadius(0.0);
        start->setResolution(1);
        mpControls->mpProbDefControl->addStartRegion(std::move(start));
        mpControls->mpPlanningController->updateProblemDefinition(name);
        using namespace std::chrono;
        const auto t0 = high_resolution_clock::now();
        const long timeMax = 3000l;
        long timeSpend(0l);
        do
        {
          mpControls->mpPlanningController->createPaths(name, 1.0);
          timeSpend = duration_cast<milliseconds>( high_resolution_clock::now()-t0 ).count();
        } 
        while (timeSpend < timeMax && coll.getPaths().size() == 0);
      }

      if (coll.getPaths().size() == 0)
        throw MUK_EXCEPTION_SIMPLE("Could not find a feasible plan during replanning");
      {
        // atm: the original is shown in green. The following makes sure the currently replanned is clearly visible and distinguishable from the original.
        auto pVisPath = mpModels->pVisModel->getVisScene()->getPathCollection(name)->getMukPath(0);
        pVisPath->setLineWidth(5.0);
        pVisPath->setColors(Vec3d(Colors::Orange));
        mpControls->mpVisControl->showOnlyPath(name, 0);
      }
      MukPath newPath = coll.getPaths()[0];
      // / ==== reimplemente this
      emit mWorkerInterface.requestUpdatePath(newPath);
      emit mWorkerInterface.requestProceed();
    }

    /**
    */
    void NavigationThread::updateAdaptedPath(const gris::muk::MukPath& path)
    {
      mpModels->pVisModel->setAdaptedPath(path);
    }

    /**
    */
    void NavigationThread::updatedCurrentTargetOrientation(const gris::Vec3d& orientation)
    {
      mpModels->pVisModel->setCurrentTargetOrientation(orientation);
    }

    /**
    */
    void NavigationThread::updateCoordinateSystem(const gris::muk::MukTransform& transform)
    {
      mpModels->pVisModel->updateTransform(transform);
    }

    /**
    */
    void NavigationThread::updateRobotTransform(const gris::muk::MukTransform& transform)
    {
      mpModels->pVisModel->setRobotTransform(transform);
    }

    /**
    */
    void NavigationThread::handleException(const gris::muk::MukException& e)
    {
      mpModels->pAppModel->handleException(e);
    }

    /**
    */
    void NavigationThread::handleException(const std::exception& e)
    {
      mpModels->pAppModel->handleException(e);
    }
  }
}