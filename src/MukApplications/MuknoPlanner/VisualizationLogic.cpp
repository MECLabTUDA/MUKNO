#include "private/muk.pch"
#include "private/VisualizationLogicLegacyIO.h"
#include "private/DefaultInteraction3D.h"

#include "VisualizationLogic.h"

#include "ApplicationLogic.h"
#include "PlanningLogic.h"
#include "WaypointManipulator.h"
#include "NavigationThread.h"

#include "MukCommon/PathCollection.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukStringToolkit.h"

#include "MukVisualization/CameraConfiguration.h"
#include "MukVisualization/muk_colors.h"
#include "MukVisualization/VisAbstractObject.h"
#include "MukVisualization/VisBounds.h"
#include "MukVisualization/VisCoordinateSystem.h"
#include "MukVisualization/VisMukStateRegion.h"
#include "MukVisualization/VisObstacle.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisPreliminaryTestRobot.h"
#include "MukVisualization/VisMukRobotSource.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisStateRegionFactory.h"

#include "MukVisualization/VisTrajectory.h"
#include "MukVisualization/VisVector.h"

#include "MukQt/CtWindow.h"
#include "MukQt/muk_qt_tools.h"
#include "MukQt/MukQMenuBar.h"
#include "MukQt/MukQToolBar.h"
#include "MukQt/TabNavigation.h"
#include "MukQt/VTKWindow.h"
#include "MukQt/TabWidgetVisualization.h"

#include "gstd/XmlDocument.h"
#include "gstd/XmlNode.h"

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

Q_DECLARE_METATYPE(gris::muk::MukState)
#include <memory>



namespace gris
{
namespace muk
{
	/**
  */
  VisualizationLogic::VisualizationLogic()
    : BaseLogic()
    , mpVisScene(std::make_unique<VisScene>())
		, mDefaultView(std::make_shared<CameraConfiguration>())
		, mDefaultViewSet(false)
		, mTimerId(0)
	{
		declareProperty<CameraConfiguration>("Camera",
			std::bind(&VisualizationLogic::setCameraConfiguration, this, std::placeholders::_1),
			std::bind(&VisualizationLogic::getCameraConfiguration, this));
	}

	/**
	*/
	VisualizationLogic::~VisualizationLogic()
	{
    //mpVisScene.reset();
	}

	/**
	*/
	void VisualizationLogic::setupConnections()
	{
		connect(mpLogics->pAppLogic.get(), &ApplicationLogic::obstacleAdded, this, &VisualizationLogic::addObstacle);
		connect(mpLogics->pAppLogic.get(), &ApplicationLogic::obstacleDeleted, this, &VisualizationLogic::deleteObstacle);
	  connect(mpLogics->pVisLogic->getVisWindow(), &QTabWidget::currentChanged, this, &VisualizationLogic::currentTabChanged);
	}

	/**
	*/
  void VisualizationLogic::reset()
	{
    mpVisScene->reset();
	}
  
	/**
	*/
	void VisualizationLogic::setCameraConfiguration(const CameraConfiguration& config)
	{
		*mDefaultView = config;
		mDefaultViewSet = true;
	}

  /**
  */
	const CameraConfiguration& VisualizationLogic::getCameraConfiguration() const
	{
		return *mDefaultView;
	}

	/**
	*/
	void VisualizationLogic::setScene(std::shared_ptr<MukScene> pScene)
	{
		mpScene = pScene;
	}

	/**
	*/
  void VisualizationLogic::setVisWindow(TabWidgetVisualization* pVisWindow)
	{
		mpVisWindow = pVisWindow;
    // set the renderer for CoordinateSystems display
    mCoordinateSystems.setRenderer(pVisWindow->getVtkWindow()->getRenderer());
	}

	/**
	*/
	void VisualizationLogic::addPath(const std::string& name, size_t idx)
	{
		auto& coll = mpScene->getPathCollection(name);
		const auto& paths = coll.getPaths();
		const size_t N = paths.size();
		if (N <= idx)
		{
			throw MUK_EXCEPTION((boost::format("unable to create visualization of path with idx %d") % idx).str().c_str(), (boost::format("There are only %d paths in PathCollection '%s'") % N % name).str().c_str());
	  }    
		const MukPath& path = paths[idx];
		auto pVis = std::make_shared<VisMukPath>(path);
    {
      auto* pInterpolator  = mpLogics->pAppLogic->getScene()->getInterpolator();
      pInterpolator->setInput(path);
      pInterpolator->interpolate();
      auto pathInterpolated = pInterpolator->getInterpolation();
      pVis->setData(pathInterpolated);
      pVis->setRenderer(mpVisWindow->getVtkWindow()->getRenderer());
      pVis->update();
    }
		mpVisScene->getPathCollection(name)->addMukPath(pVis);
    //mpVisWindow->getVtkWindow()->Render();
	}

	/**
	*/
  void VisualizationLogic::updatePath(const std::string& name, size_t idx)
	{
    auto& coll = mpScene->getPathCollection(name);
    auto* pInterpolator  = mpScene->getInterpolator();
		auto pObj = mpVisScene->getPathCollection(name)->getMukPath(idx);
    {
      pInterpolator->setInput(coll.getPaths()[idx]);
      pInterpolator->interpolate();
      auto pathInterpolated = pInterpolator->getInterpolation();
      pObj->setData(pathInterpolated);
      pObj->update();
    }
		mpVisWindow->getVtkWindow()->Render();
	}

	/**
	*/
  void VisualizationLogic::clearPaths(const std::string& name)
  {
    const auto N = mpVisScene->getPathCollection(name)->numberOfPaths();;
    for (size_t i(0); i<N; ++i)
    {
      deletePath(name, 0);
    }
  }

  /**
  */
	void VisualizationLogic::deletePath(const std::string& name, size_t idx)
	{
		mpVisScene->getPathCollection(name)->deleteMukPath(idx);
	}

	/**
	*/
	void VisualizationLogic::addPathCollection(const std::string& name)
	{
    auto& coll     = mpScene->getPathCollection(name);
    auto  pProbDef = coll.getProblemDefinition();
    auto  pVisColl = std::make_shared<VisPathCollection>(coll);
    size_t N = pProbDef->sizeStart();
    for (size_t i(0); i<N; ++i)
		{
      auto pObj = VisStateRegionFactory::create(pProbDef->getStartRegion(i));
      pObj->setName("StartRegion" + std::to_string(i));
      pObj->setRenderer(mpVisWindow->getVtkWindow()->getRenderer());
      pVisColl->addStartRegion(pObj);
		}
    N = pProbDef->sizeGoal();
    for (size_t i(0); i<N; ++i)
    {
      auto pObj = VisStateRegionFactory::create(pProbDef->getGoalRegion(i));
      pObj->setName("GoalRegion" + std::to_string(i));
      pObj->setRenderer(mpVisWindow->getVtkWindow()->getRenderer());
      pVisColl->addGoalRegion(pObj);
    }
    N = pProbDef->sizeWaypoints();
    auto waypoints = pProbDef->getWaypoints();
    for (size_t i(0); i<N; ++i)
    {
      auto pObj = std::make_shared<VisMukStateRegion>("WP" + std::to_string(i));
      pObj->setState(waypoints[i]);
      pObj->setPhi(0);
      pObj->setRadius(1);
      pObj->setRenderer(mpVisWindow->getVtkWindow()->getRenderer());
      pVisColl->addWaypoint(pObj);
    }
    const auto& paths = coll.getPaths();
    for (const auto& path : paths)
    {
      auto pObj = std::make_shared<VisMukPath>(path);
      pObj->setRenderer(mpVisWindow->getVtkWindow()->getRenderer());
      pObj->update();
      pVisColl->addMukPath(pObj);
    }

    mpVisScene->addPathCollection(pVisColl);
		mpVisWindow->getVtkWindow()->Render();
	}

	/**
	*/
	void VisualizationLogic::deletePathCollection(const std::string& name)
	{
		mpVisScene->deletePathCollection(name);
		mpVisWindow->getVtkWindow()->Render();
	}

	/**
	*/
	void VisualizationLogic::addObstacle(const std::string& name)
	{
		const auto obj = mpScene->getObstacle(name);
		auto       pVis = std::make_shared<VisObstacle>(*obj);
		pVis->setRenderer(mpVisWindow->getVtkWindow()->getRenderer());
    pVis->setData(obj->getData());
		mpVisScene->addObstacle(pVis);
	}

	/**
	*/
	void VisualizationLogic::deleteObstacle(const std::string& name)
	{
		mpVisScene->deleteObstacle(name);
		mpVisWindow->getVtkWindow()->Render();
	}

	/**
	*/
	void VisualizationLogic::addAbstractObject(std::shared_ptr<VisAbstractObject> pObj)
	{
    auto keys = getAbstractObjectKeys();
    if (std::any_of(keys.begin(), keys.end(), [&] (const auto& key) { return key == pObj->getName(); }))
      throw MUK_EXCEPTION("key already exists", pObj->getName().c_str());

		pObj->setRenderer(mpVisWindow->getVtkWindow()->getRenderer());
    mpVisScene->addObject(pObj);
	}

	/**
	*/
	void VisualizationLogic::deleteAbstractObject(const std::string& name)
	{
	  mpVisScene->deleteObject(name);
	}

  /**
  */
  void VisualizationLogic::clearAbstractObjects()
  {
    auto keys = mpVisScene->getAbstractObjectKeys();
    for (const auto& key : keys)
    {
      mpVisScene->deleteObject(key);
    }
  }

  /**
  */
  void VisualizationLogic::updateTransform(const MukTransform& transform)
  {
    mCoordinateSystems.updateCoordinateSystem(transform);
  }

	/** \brief Slot for Navigation to call on a change of the internal Initialization State
	*/
	void VisualizationLogic::updateNavState_Initialization(const bool pState)
	{
    // from VisualizationLogic::initNaviVisualization()
    if (pState)
		{
			// check whether a robot is available / has been set
      auto keys = getAbstractObjectKeys();
      //const std::string robotName = "VisPreliminaryTestRobot";
      const std::string robotName = "VisMukRobotSource";
      if ( ! std::any_of(keys.begin(), keys.end(), [&] (const auto& str) { return robotName == str;}))
		  {
			  //setRobot("VisPreliminaryTestRobot");
        setRobot("VisMukRobotSource");
      }
      else
      {
        mpVisScene->getRobot().loadModel();
      }
      const std::string pathName = "AdaptedPath";
      if ( ! std::any_of(keys.begin(), keys.end(), [&] (const auto& str) { return pathName == str;}))
      {
        auto pTrajectory = std::make_shared<VisTrajectory>(pathName);
				pTrajectory->setDefaultColor(Colors::Magenta);
				addAbstractObject(pTrajectory);
			}
      const std::string targetVecName = "CurrentTargetOrientation";
      if (!std::any_of(keys.begin(), keys.end(), [&](const auto& str) { return targetVecName == str; }))
      {
        auto pVectorTrajectory = std::make_shared<VisVector>(targetVecName);
        pVectorTrajectory->setDefaultColor(Colors::Cyan);
        addAbstractObject(pVectorTrajectory);
      }
		}
		else
		{
			LOG_LINE << "WARNING: resetting the initialization state is not yet supported.";
		}
	}

	/** \brief Slot for Navigation to call on a change of the internal Running State
	*/
	void VisualizationLogic::updateNavState_Running(const bool pState)
	{
		// add a Timer, if the "activate", remove, if we deactivate
		if (pState) addTimer();
		else removeTimer();
	}

	/** \brief initialize timed event at currently 500 milliseconds
	*/
	void VisualizationLogic::addTimer()
	{
		if (mTimerId != 0)
		{
			removeTimer();
		}
		mTimerId = startTimer(500, Qt::PreciseTimer);
	}

	/** \brief Remove timed event
	*/
	void VisualizationLogic::removeTimer()
	{
    killTimer(mTimerId);
		mTimerId = 0;
	}

	/** \brief Implement timed Event as single call to this->render()
	*/
	void VisualizationLogic::timerEvent(QTimerEvent* e)
	{
    LOG_LINE << __FUNCTION__;
    // update the Robot Position and Orientation (State)
	  auto& pRobot = mpVisScene->getRobot();
    pRobot.setTransform( mpRobotTransform );

    // update the 'future trajectory'
    auto* pTrajectory = dynamic_cast<VisTrajectory*>(mpVisScene->getObject("AdaptedPath").get());
    pTrajectory->setData( mAdaptedPath );
    pTrajectory->update();

    // update the current targetorientation state
    auto* pTargetOrient = dynamic_cast<VisVector*>(mpVisScene->getObject("CurrentTargetOrientation").get());
    auto position = Vec3d(mpRobotTransform->GetPosition());
    pTargetOrient->setVector(position, mCurrentTargetOrientation);
    pTrajectory->update(); // this essentially does nothing

    // if debugging is enabled, update the debugging vectors, the text needs extra updates.
    mCoordinateSystems.updateBeforeRender();
		render();
	}

	/** \brief Show rectangle with min/max of Planning Bounds
	*/
	void VisualizationLogic::showBounds()
	{
		const auto& key = mpLogics->pPlanningLogic->getActivePathCollection();
    if ( ! key.empty())
    {
      const auto& bounds = mpScene->getPathCollection(key).getProblemDefinition()->getBounds();
      auto* pObj = dynamic_cast<VisBounds*>(mpVisScene->getObject("Bounds").get());
      pObj->setBounds(bounds);
    }
    else
    {
      LOG_LINE << "Bounds are only available if a path collection exists";
    }
	}

	/**
	*/
	void VisualizationLogic::setDefaultFocus()
	{
		if (!mpVisWindow)
		{
			throw MUK_EXCEPTION_SIMPLE("VisualizationWindow not set!");
		}
		*mDefaultView = mpVisWindow->getVtkWindow()->getCameraConfiguration();
		mDefaultViewSet = true;
	}

	/**
	*/
	void VisualizationLogic::render()
	{
		mpVisWindow->getVtkWindow()->Render();
	}

	/**
	*/
	void VisualizationLogic::save(const std::string& filename) const
	{
		namespace fs = boost::filesystem;
		fs::path fn = filename;
    if (!fs::is_regular_file(fn))
			throw MUK_EXCEPTION("File does not exist!", fn.string().c_str());

		mpVisScene->save(filename);

		auto pDoc = gris::XmlDocument::read(fn.string().c_str());
		auto root = pDoc->getRoot();
		auto node = root.getChild("SurgeryPlanning").getChild("Configuration").addChild("Visualization");
		// vtkWindow
		{
			auto config = mDefaultViewSet ? mpVisWindow->getVtkWindow()->getCameraConfiguration() : *mDefaultView;
			auto subNode = node.addChild("Camera");
			auto subsubNode = subNode.addChild("FocalPoint");
			{
				std::stringstream ss;
        ss << config.getFocalPoint();
				subsubNode.setValue(ss.str().c_str());
			}
			subsubNode = subNode.addChild("Position");
			{
				std::stringstream ss;
        ss << config.getPosition();
				subsubNode.setValue(ss.str().c_str());
			}
			subsubNode = subNode.addChild("ViewUp");
			{
				std::stringstream ss;
        ss << config.getViewUp();
				subsubNode.setValue(ss.str().c_str());
			}
		}
		gris::XmlDocument::save(fn.string().c_str(), *pDoc);
	}

	/** \brief Focuses on DefaultView, if not available tries to focus on an obstacle or on a ProblemDefinition. Else focuses on (0,0,0).
      
	*/
	void VisualizationLogic::focus()
	{
		if (mDefaultViewSet)
		{
			mpVisWindow->getVtkWindow()->setCameraConfiguration(*mDefaultView);
		}
    else
    {
      auto keys = mpScene->getObstacleKeys();
      if (!keys.empty())
      {
        auto pObj = mpScene->getObstacle(keys.front());
        double pd[6];
        pObj->getData()->GetBounds(pd);
        mpVisWindow->getVtkWindow()->setFocalPoint(0.5*Vec3d(pd[0]+pd[1], pd[2]+pd[3], pd[4]+pd[5]));
      }
      else
      {
        keys = mpScene->getPathKeys();
        if (keys.empty())
        {
          mpVisWindow->getVtkWindow()->setFocalPoint(Vec3d(0.0));
        }
        else
        {
          const auto pProb = mpVisScene->getPathCollection(keys.front());
          if ( 0 != pProb->sizeStart())
            mpVisWindow->getVtkWindow()->setFocalPoint(pProb->getStartRegion(0).getCenter().coords);
          else if ( 0 != pProb->sizeGoal())
            mpVisWindow->getVtkWindow()->setFocalPoint(pProb->getGoalRegion(0).getCenter().coords);
          else
            mpVisWindow->getVtkWindow()->setFocalPoint(Vec3d(0.0));
        }
      }
    }
    mpVisWindow->getVtkWindow()->Render();
	}
		
  /**
  */
  void VisualizationLogic::focusOnWaypoint()
  {
    int index = mpLogics->pWpManipulator->getIndex();
    if (index < 0)
      return;
    auto& pVisColl = mpVisScene->getPathCollection(mpLogics->pWpManipulator->getKey());
    const auto newFocalPoint = pVisColl->getStartRegion(0).getCenter().coords;
    mpVisWindow->getVtkWindow()->setFocalPoint(newFocalPoint);
    mpVisWindow->getVtkWindow()->Render();
  }

  /**
  */
  void VisualizationLogic::synchronize(const MukScene& scene)
  {
    auto scenekeys = scene.getObstacleKeys();
    auto viskeys   = mpVisScene->getObstacleKeys();
    for (const auto& key : scenekeys)
    {
      if (std::none_of(viskeys.begin(), viskeys.end(), [&] (const auto& str) { return key == str; }))
      {
        addObstacle(key);
      }
    }
    // synchronize pathcollections
    scenekeys = scene.getPathKeys();
    viskeys   = mpVisScene->getPathCollectionKeys();
    for (const auto& key : scenekeys)
    {
      if (std::none_of(viskeys.begin(), viskeys.end(), [&] (const auto& str) { return key == str; }))
      {
        addPathCollection(key);
      }
    }
  }

  /** \brief loads the visualization data for a VisScene from a xml-file (*.mukno)

  The obstacles and paths should all already exist in the scene.
  To create a VisScene from MukScene call synchronize()
	*/
	void VisualizationLogic::load(const std::string& filename)
	{
		namespace fs = boost::filesystem;
		fs::path p = filename;
		if (!fs::is_regular_file(p))
			throw MUK_EXCEPTION("File does not exist!", p.string().c_str());
		auto pDoc = gris::XmlDocument::read(p.string().c_str());
		gris::XmlNode root = pDoc->getRoot();

	  if ( ! VisualizationLogicLegacyIO::load(root, mpScene.get(), mpVisScene.get(), this))
		{
			mpVisScene->load(filename);
      auto node = pDoc->getRoot().getChild("SurgeryPlanning").getChild("Configuration").getChild("Visualization");
      {
        CameraConfiguration config;
        auto subNode = node.getChild("Camera");
        auto subsubNode = subNode.getChild("FocalPoint");
        auto iss = std::istringstream(subsubNode.getValue());
        Vec3d tmp;
        iss >> tmp;
        config.setFocalPoint(tmp);
        subsubNode = subNode.getChild("Position");
        iss = std::istringstream(subsubNode.getValue());
        iss >> tmp;
        config.setPosition(tmp);
        subsubNode = subNode.getChild("ViewUp");
        iss = std::istringstream(subsubNode.getValue());
        iss >> tmp;
        config.setViewUp(tmp);
        setCameraConfiguration(config);
      }
      focus();
    }
	}

	/**
	*/
	std::vector<std::string> VisualizationLogic::getAbstractObjectKeys() const
	{
		return mpVisScene->getAbstractObjectKeys();
	}

	/** \brief sets the robot visualization

	this logic has to be adjusted as soon as changing between multiple robots is possible
	*/
	void VisualizationLogic::setRobot(const std::string& name)
	{
		//auto pObj = std::make_shared<VisPreliminaryTestRobot>();
    auto pObj = std::make_shared<VisMukRobotSource>();
    addAbstractObject(pObj);    
    mpVisScene->setRobot(pObj);
    pObj->setState(MukState(Vec3d(15,0,15), Vec3d(0,1,0)));
	}

	/**
	*/
	void VisualizationLogic::setRobotState(const std::string& pathCollection, size_t pathIdx, size_t stateIdx)
	{
		auto& coll = mpScene->getPathCollection(pathCollection);
		size_t N = coll.getPaths().size();
		if (N <= stateIdx)
		{
			std::string str1 = (boost::format("Requested Path-Index does not exist in the Pathcollection")).str();
			std::string str2 = (boost::format("PathCollection '%s' has only %d Paths. Index %d was requested") % pathCollection % N % pathIdx).str();
			throw MUK_EXCEPTION(str1.c_str(), str2.c_str());
		}

		auto path = mpScene->getInterpolator()->getInterpolation();

		N = path.getPath().size();
		if (stateIdx >= N)
		{
			std::string str1 = (boost::format("Requested State-Index does not exist in the Path")).str();
			std::string str2 = (boost::format("PathCollection '%s', Path %d: Only %d States available. Index %d was requested") % pathCollection % pathIdx % N % stateIdx).str();
			throw MUK_EXCEPTION(str1.c_str(), str2.c_str());
		}
		auto& pObj = mpVisScene->getRobot();
		pObj.setState(path.getPath()[stateIdx]);
		render();
	}

	/** \brief When the tab is changed to the VtkWindow then the visual objects are updated.
	*/
	void VisualizationLogic::currentTabChanged(int index)
	{
    if (index == TabWidgetVisualization::enView3D)
		{
      auto keys = getAbstractObjectKeys();
      std::vector<std::string> keysToDelete = { "distanceLine", "3dAxes", "zPlane", "xPlane", "yPlane" };

      for (auto& s : keysToDelete)
      {
        if (std::find(keys.begin(), keys.end(), s) != keys.end())
        {
          deleteAbstractObject(s);
        }
      }
      render();
		}
	}

  /** \brief Qt slot to update the Robot Transform visualization variable

    This could also trigger an update of the rendering.
		  */
  void VisualizationLogic::setRobotTransform(const MukTransform& robotTransform)
  {
    mpRobotTransform = robotTransform.getTransform();
  }

  /** \brief Qt slot to update the current target Orientatino visualization variable

  This could also trigger an update of the rendering.
		*/
  void VisualizationLogic::setCurrentTargetOrientation(const Vec3d& currentTargetOrientation)
	{
    mCurrentTargetOrientation = currentTargetOrientation;
	}

  void VisualizationLogic::updateNavigatorFeature(const INavigator::NavigatorFeature & feature, const bool& state )
	{
    if (feature == INavigator::updateCoordinateSystem && state == false)
      mCoordinateSystems.clearCoordinateSystems();
	}

  /** \brief Qt slot to update the adapted path visualization variable

  This could also trigger an update of the rendering.
		*/
  void VisualizationLogic::setAdaptedPath(const MukPath& adaptedPath)
	{
    mAdaptedPath = adaptedPath;
	}

}
}