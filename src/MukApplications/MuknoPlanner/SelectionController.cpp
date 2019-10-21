#include "private/muk.pch"
#include "SelectionController.h"
#include "ToolBarController.h"
#include "AppControllers.h"
#include "PlanningController.h"
#include "PropertyController.h"
#include "VisualizationController.h"
#include "InteractionController.h"
#include "MenuBarController.h"

#include "private/muk.pch"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/SelectionModel.h"
#include "MukAppModels/VisualizationModel.h"
#include "MukAppModels/WorldVisualizationModel.h"

#include "MukCommon/MukObstacle.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/vtk_tools.h"

#include "MukEvaluation/statistics.h"
#include "MukEvaluation/CollisionDetectionHandler.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/VisImageOverlay.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisObstacle.h"

#include "MukQt/muk_qt_tools.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/MukQToolBar.h"
#include "MukQt/ParetoWidget.h"
#include "MukQt/QuickPathAnalysisWindow.h"
#include "MukQt/SceneWidget.h"
#include "MukQt/TabSelection.h"
#include "MukQt/TabPlanning.h"
#include "qcustomplot.h"
#include "qevent.h"

#include "vtkPolyData.h"
#include "PolyDataHandler.h"
#include "PolyDataMapperHandler.h"
#include "vtkCleanPolyData.h"
#include "vtkTubeFilter.h"


namespace
{
  using namespace gris::muk;

  /** special class for the tabSelectionWindow since the normal sends no signals when closed with the X button or alt+F4
  */
  class TabSelectionWindow : public QMainWindow
  {
    public:
      TabSelectionWindow(QWidget *parent = 0);

    public:
      void closeEvent(QCloseEvent *closeEvent);
  };

  TabSelectionWindow::TabSelectionWindow(QWidget *parent) 
    : QMainWindow(parent)
  {
  }

  // sends a signal when manually closed
  void TabSelectionWindow::closeEvent(QCloseEvent *closeEvent)
  {
    emit TabSelectionWindow::destroyed();
    closeEvent->accept();
  }

  /** pareto fron dummies
  */
  const std::string Invalid_Choice  = "- not selected -";
  const std::string Path_Length     = "Path Length";
  const std::string Min_Distance    = "Minimum Distance";
}

namespace gris
{
  namespace muk
  {

    const char* OverlayImageName = "OverlayImage";
    const double componentWeightsInitValue = 0.25;
    const double componentFiltersInitValue = 100;
    const double obstacleWeightsInitValue = 0.25;
    const double obstacleFiltersInitValue = 0.5;
    const size_t maxObstacleCount = 12;
    size_t componentCount = 6;
    size_t obstacleCount = 12;
    size_t accessCanalCount = 3;

    /**
    */
    SelectionController::SelectionController()
      : mOverlayIsActive(false)
    {
    }

    /**
    */
    SelectionController::~SelectionController()
    {
    }

    /**
    */
    void SelectionController::initialize()
    {
      myModel = mpModels->pSelectionModel.get();
    }

    /**
    */
    void SelectionController::setupConnections()
    {
      auto* pTabSelection = mpMainWindow->mpTabSelection;
      // when the load ct-file button is clicked and smth is loaded
      connect(mpControls->mpToolbar.get(), &ToolBarController::ctFileLoaded, this, &SelectionController::ctFileLoaded);
      // when the clear scene button is clicked or the planning in the planning controller has changed the whole tab will be resettet
      connect(mpControls->mpToolbar.get(), &ToolBarController::sceneCleared, this, [=] { mActiveKey.clear();  resetTabSelection(); });
      connect(mpControls->mpPlanningController.get(), &PlanningController::planningChanged, this, &SelectionController::resetTabSelection);
      // when the activity of a property (an obstacle) is changed the model will be recalculated
      connect(mpControls->mpPropControl.get(), &PropertyController::activChanged, this, &SelectionController::recalculateModel);
      // when the tab switches the selection tab is reloaded (if it switched to the tab and an update is needed)
      connect(mpMainWindow->mpTabContainer, &QTabWidget::currentChanged, this, &SelectionController::updateSelectionTab);
      
      connect(mpMainWindow->mToolBar, &MukQToolBar::pathAnalysisRequested, this, &SelectionController::quickPathAnalysis);
      //Connections for TabSelection
      {
        // managing the advanced options display
        connect(pTabSelection, &TabSelection::enlargeClicked, this, &SelectionController::advancedOptionsEnlarge);
        connect(pTabSelection, &TabSelection::windowClicked, this, &SelectionController::advancedOptionsWindow);
        // the quick-Selection buttons
        connect(pTabSelection, &TabSelection::selectLargestDistanceClicked, this, &SelectionController::highlightLargestDistance);
        connect(pTabSelection, &TabSelection::selectStraightestPathClicked, this, &SelectionController::highlightStraightesPath);
        connect(pTabSelection, &TabSelection::selectBestAnglePathClicked, this, &SelectionController::highlightSmallestGoalAngle);
        connect(pTabSelection, &TabSelection::selectShortestPathClicked, this, &SelectionController::highlightShortestPath);
        connect(pTabSelection, &TabSelection::selectLeastThickBoneClicked, this, &SelectionController::highlightLeastThickBone);
        connect(pTabSelection, &TabSelection::selectShortestAirHoleClicked, this, &SelectionController::highlightShortestAirHole);
        // single selection and show only
        connect(pTabSelection, &TabSelection::singlePathSelectionChanged, this, &SelectionController::singlePathSelectionChanged);
        connect(pTabSelection, &TabSelection::showOnlyClicked, this, SELECT<void>::OVERLOAD_OF(&SelectionController::showSinglePath));
        // color paths and reset selection
        connect(pTabSelection, &TabSelection::colorPathsClicked, this, &SelectionController::colorPaths);
        connect(pTabSelection, &TabSelection::resetSelectionClicked, this, &SelectionController::resetSelection);
        // everything that has to do with the ctOverlay
        connect(pTabSelection, &TabSelection::displayedStateChanged, this, &SelectionController::ctStateChanged);
        connect(pTabSelection, &TabSelection::ctOverlayScrolled, this, &SelectionController::scrollCTOverlay);
        connect(pTabSelection, &TabSelection::ctOverlayClicked, this, &SelectionController::toggleCTOverlay);
        // everything regarding accessCanals
        connect(pTabSelection, &TabSelection::indexChosenClicked, this, &SelectionController::selectAccessCanal);
        connect(pTabSelection, &TabSelection::cutOffDistanceChanged, this, &SelectionController::cutOffDistanceChanged);
        connect(pTabSelection, &TabSelection::asObstacleClicked, this, &SelectionController::setCanalAsObstacle);
        connect(pTabSelection, &TabSelection::fillCanalsClicked, this, &SelectionController::autoFillCanals);
        // everything regarding the paretoFront
        connect(pTabSelection, &TabSelection::paretoFrontClicked, this, &SelectionController::showParetoFrontWindow);
        connect(pTabSelection, &TabSelection::requestParetoFrontReset, this, &SelectionController::resetParetoFront);
        //connect(pTabSelection, &TabSelection::paretoExitClicked, this, &SelectionController::exitParetoFrontWindow);
        //connect(pTabSelection, &SelectionController::parameterChosen, this, &SelectionController::paretoParameterChosen);
        // everything interesting for weighting and evaluation
        // when a weighting is changed the path will be automaticly evaluated afterwards
        connect(pTabSelection, &TabSelection::componentWeightingChanged, this, [=] {componentWeightingChanged(); evaluatePaths(); });
        connect(pTabSelection, &TabSelection::obstacleWeightingChanged, this, [=] {obstacleWeightingChanged(); evaluatePaths(); });
        connect(pTabSelection, &TabSelection::evaluatePathClicked, this, &SelectionController::evaluatePaths);
        // everything there is for filtering
        // when a filter is changed both obstacle filter and component filter will be called because they can influence each other
        // after obstacle filter changed is called the filtered paths will be set and the filterPaths function will be called
        connect(pTabSelection, &TabSelection::componentFilterChanged, this, &SelectionController::componentFilterChanged);
        connect(pTabSelection, &TabSelection::obstacleFilterChanged, this, [=](int ind) {obstacleFilterChanged(ind); mfilteredPaths = myModel->filterPaths(); filterPaths(); });
        connect(pTabSelection, &TabSelection::filterPathsClicked, this, [=] {mfilteredPaths = myModel->filterPaths(); filterPaths(); });
      }
    }

    /**
    */
    void SelectionController::finalize()
    {
      if(mpPlot)
        mpPlot->close();
      windowClosed();
    }

    /**
    */
    void SelectionController::updateSelectionTab()
    { // when the current Tab is the selection tab or the tab is in the window it will update
      if ( mpMainWindow->mpTabContainer->currentIndex() == MuknoPlannerMainWindow::enSelection
        || mTabIsWindow)
      { 
        // just needed when the tab is windowed or enlarged before a scene was loaded (gives an error if not for this passage)
        // calculates the model if needed
        selectPathCollection();
        // updates the selectionTab UI
        mpMainWindow->mpTabSelection->updateComponentCount(mCTFileLoaded, !(mTabIsLarge || mTabIsWindow));
        updateTabSelectionObstacles();
        componentFilterChanged(-1);
        obstacleFilterChanged(-1);
        myModel->filterPaths();
        componentFilterChanged(-1);
        obstacleFilterChanged(-1);
      }
      else
      { // when the current tab is not the selection tab it is shrinked back to the normal size and the overlay (if active) will be removed
        if (mOverlayIsActive)
          toggleCTOverlay();
        if (mTabIsLarge)
          advancedOptionsEnlarge();
      }
    }

    /** \brief load the selection of the path collection or create a new one and calculate the model if requested
    */
    void SelectionController::selectPathCollection()
    {
      mActiveKey = mpModels->pPlanningModel->getActivePathCollection();
      if(mActiveKey.empty())
        return;
      const auto& coll = mpModels->pAppModel->getScene()->getPathCollection(mActiveKey);
      if (myModel->hasSelection(mActiveKey))
      {
        myModel->makeSelectionValid(mActiveKey, coll.getPaths().size());
      }
      else
      {
        myModel->makeSelection(mActiveKey);
      }
      myModel->loadPathCollection(mActiveKey);
      mpMainWindow->mpTabSelection->setMaxPathIndex(coll.getPaths().size() - 1);
      if ( ! coll.getPaths().empty())
      {
        myModel->loadPath(0);
      }
      else
      {
        myModel->loadPath(-1);
      }
      if (mTabNeedsUpdate)
      {
        myModel->compute();
        mTabNeedsUpdate = false;
      }
    }

    /** \brief removes all selected indices from the active path collecton's selection, adjusts the view accordingly
    */
    void SelectionController::resetSelection()
    {
      if (mActiveKey.empty())
        return;
      setDefaultSelectionView();
      auto accessCanals = myModel->selectedIndices();
      // Unsets all AccessCanals that are not set as an obstacle (or already unset)
      for (size_t i(0); i < accessCanals.size(); i++)
      {
        if (!mSetAsObstacle[i] && accessCanals[i] != -1)
        {
          myModel->setAccessCanal(i, accessCanals[i]);
          mpMainWindow->mpTabSelection->setAccessCanalIndex(i, -1);
          mpMainWindow->mpTabSelection->updateCutOffDistance(i, 1);
        }
      }
      mpMainWindow->mpTabSelection->setSinglePathSelection(-1);
      mpMainWindow->mpTabSelection->checkAccessCanals(myModel->selectedIndices());
      mPathIsColored = false;
      mpModels->pVisModel->render();
      updateSelectionTab();
    }

    /**
    */
    void SelectionController::showSinglePath()
    {
      size_t idx = mpMainWindow->mpTabSelection->getSinglePathSelection();
      showSinglePath(idx);
    }

    /**
    */
    void SelectionController::showSinglePath(size_t idx)
    {
      setDefaultSelectionView();
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      auto pColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
      const auto N = pColl->numberOfPaths();
      for (size_t i(0); i<N; ++i)
      {
        if (i != idx)
        {
          pColl->getMukPath(i)->setVisibility(false);
        }
      }
      mPathIsColored = false;
      mpModels->pVisModel->render();
    }

    /** \brief hide all other path collections (and make the active one visible again)
    */
    void SelectionController::showOnlyCollection()
    {
      const auto keys = mpModels->pVisModel->getVisScene()->getPathCollectionKeys();
      for (const auto& key : keys)
      {
        setCollectionVisibility(key, key == mActiveKey);
      }
      mpModels->pVisModel->render();
    }

    /** \brief color codes the paths according to the distance to risk structures

    todo: should perform color coding according to different color maps
    */
    void SelectionController::colorPaths()
    {
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
      const auto N = pVisColl->numberOfPaths();
      const auto pScene = mpModels->pAppModel->getScene();
      const auto& coll = pScene->getPathCollection(key);
      double r = coll.getProblemDefinition()->getRadius();
      if (N == 0)
        return;

      const auto& v = myModel->getDistanceOrder();

      double color[3];
      const double N_2 = N*0.5;
      // from green to orange
      for (size_t i(0); i<static_cast<size_t>(N_2); ++i)
      {
        color[0] = std::min(i / N_2, 1.0); // 0 at i=0; 1 at i=N_2
        color[1] = Colors::Green[1];
        color[2] = Colors::Green[2];
        pVisColl->getMukPath(v[i])->setColors(color);
      }

      // from orange to red
      for (size_t i(static_cast<size_t>(N_2)); i<N; ++i)
      {
        color[0] = Colors::Red[0];
        color[1] = abs(1.0*(N - i)) / N_2; // 1 at i=N_2, 0 at i=N
        color[2] = Colors::Red[2];
        pVisColl->getMukPath(v[i])->setColors(color);
      }
      mPathIsColored = true;
      mpModels->pVisModel->render();
    }

    /** when changing the spinbox in tabSelection the ind is checked if it's viable and shown if is
    */
    void SelectionController::singlePathSelectionChanged(size_t ind)
    {
      if (ind == -1)
        highlightSinglePath(-1);
      bool indexViable = false;
      mfilteredPaths = myModel->getFilteredPaths();
      // path is viable if part of the filtered Paths
      if (std::find(mfilteredPaths.begin(), mfilteredPaths.end(), ind) != mfilteredPaths.end())
        indexViable = true;
      if (indexViable)
      {
        highlightViaSpinBox(ind);
        displayCTSlice();
      }
      else // checks if ind is the index of any of the accessCanals and sets the text of the buttons depending on that
        mpMainWindow->mpTabSelection->checkAccessCanals(myModel->selectedIndices());
    }

    /**
    */
    void SelectionController::highlightViaSpinBox(size_t idx)
    {
      LOG << "index " << idx << "\n";
      const auto& v1 = myModel->getDistances();
      if (!v1.empty())
      {
        LOG << "  minimum distance:  " << v1[idx] << "\n";

        const auto& v2 = myModel->getCurvatures();
        LOG << "  sum of curvatures: " << v2[idx] << "\n";

        const auto& v3 = myModel->getGoalAngles();
        LOG << "  angle difference:  " << v3[idx] << "\n";

        const auto& v4 = myModel->getLengths();
        LOG << "  length:            " << v4[idx] << "\n";

        if (mCTFileLoaded) {
          const auto& v5 = myModel->getBoneThickness();
          LOG << "  bone thickness:    " << v5[idx] << "\n";

          const auto& v6 = myModel->getAirHoles();
          LOG << "  longest airhole:   " << v6[idx] << "\n";
        }
        LOG_LINE;
      }
      highlightSinglePath(idx);
      mpModels->pSelectionModel->loadPath(idx);
    }

    /**
    */
    void SelectionController::highlightLargestDistance()
    {
      if (mActiveKey.empty() || mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      else
      {
        const auto& key = mpModels->pPlanningModel->getActivePathCollection();
        const auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
        const auto N = pVisColl->numberOfPaths();
        if (N == 0)
          return;

        auto result = myModel->getCurrentBest();
        LOG_LINE << "Largest minimum distance:: " << result.distance.second << " at index " << result.distance.first;
        highlightSinglePath(result.distance.first);
      }
    }

    /**
    */
    void SelectionController::highlightStraightesPath()
    {
      if (mActiveKey.empty() || mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      else
      {
        const auto& key = mpModels->pPlanningModel->getActivePathCollection();
        const auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
        const auto N = pVisColl->numberOfPaths();
        if (N == 0)
          return;

        auto result = myModel->getCurrentBest();
        LOG_LINE << "Minimum curvature:: " << result.curvature.second << " at index " << result.curvature.first;
        highlightSinglePath(result.curvature.first);
      }
    }

    /**
    */
    void SelectionController::highlightSmallestGoalAngle()
    {
      if (mActiveKey.empty() || mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      else
      {
        const auto& key = mpModels->pPlanningModel->getActivePathCollection();
        const auto pScene = mpModels->pAppModel->getScene();
        const auto& paths = pScene->getPathCollection(key).getPaths();
        const auto N = paths.size();
        if (N == 0)
          return;

        auto result = myModel->getCurrentBest();
        LOG_LINE << "Minimum goal angle difference: " << result.angle.second << " at index " << result.angle.first;
        highlightSinglePath(result.angle.first);
      }
    }

    /**
    */
    void SelectionController::highlightShortestPath()
    {
      if (mActiveKey.empty() || mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      else
      {
        const auto& key = mpModels->pPlanningModel->getActivePathCollection();
        const auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
        const auto N = pVisColl->numberOfPaths();
        if (N == 0)
          return;

        auto result = myModel->getCurrentBest();
        LOG_LINE << "Minimum path length: " << result.length.second << " at index " << result.length.first;
        highlightSinglePath(result.length.first);
      }
    }

    /**
    */
    void SelectionController::highlightLeastThickBone()
    {
      if (mActiveKey.empty() || mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      else
      {
        const auto& key = mpModels->pPlanningModel->getActivePathCollection();
        const auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
        const auto N = pVisColl->numberOfPaths();
        if (N == 0)
          return;

        auto result = myModel->getCurrentBest();
        LOG_LINE << "Minimum mean bone thickness: " << result.boneThickness.second << " at index " << result.boneThickness.first;
        highlightSinglePath(result.boneThickness.first);
      }
    }

    /**
    */
    void SelectionController::highlightShortestAirHole()
    {
      if (mActiveKey.empty() || mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      else
      {
        const auto& key = mpModels->pPlanningModel->getActivePathCollection();
        const auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
        const auto N = pVisColl->numberOfPaths();
        if (N == 0)
          return;

        auto result = myModel->getCurrentBest();
        LOG_LINE << "Minimum airhole-length: " << result.airhole.second << " at index " << result.airhole.first;
        highlightSinglePath(result.airhole.first);
      }
    }

    /**
    */
    void SelectionController::highlightSinglePath(int idx)
    {
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      auto pColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
      // make only this path visible
      const auto N = pColl->numberOfPaths();
      for (size_t i(0); i<N; ++i)
      {
        auto pPath = pColl->getMukPath(i);
        if (i == idx)
        {
          pPath->setOpacity(1.0);
          pPath->setLineWidth(5.0);
          if(!mPathIsColored)
            pPath->setColors(Vec3d{ 0,1,0 });
          // make sure, it is visible
          pPath->setVisibility(true);
        }
        else
        {
          if(!mPathIsColored)
            pPath->setColors(pPath->getDefaultColor());
          pPath->setOpacity(0.33);
          pPath->setLineWidth(1.0);
        }
      }
      mpMainWindow->mpTabSelection->setSinglePathSelection(idx);
      mpMainWindow->mpTabSelection->checkAccessCanals(myModel->selectedIndices());
      mpModels->pPlanningModel->setActivePathIdx(idx); // TODO remove this dependency
      mpModels->pVisModel->render();
    }

    /** gets the filtered paths from the model and logs which are filtered (if any)
    */
    void SelectionController::filterPaths()
    {
      const auto& key = mpModels->pPlanningModel->getActivePathCollection();
      auto pColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
      const auto N = pColl->numberOfPaths();
      if(N == 0)
        throw MUK_EXCEPTION_SIMPLE("The active PathCollection is empty");
      for (size_t i(0); i<N; ++i)
      {
        auto pPath = pColl->getMukPath(i);
        // all paths are hidden
        pPath->setVisibility(false);
      }
      if (mfilteredPaths.size() > 0)
      {
        std::string ret = "Paths that satisfy the filters are: ";
        for (size_t i(0); i < mfilteredPaths.size(); i++)
        {
          auto pPath = pColl->getMukPath(mfilteredPaths[i]);
          // the paths that satisfy the filters are revealed again
          pPath->setVisibility(true);
          if (i != 0)
            ret += ", ";
          ret  += std::to_string(mfilteredPaths[i]);
        }
        LOG_LINE << ret;
      }
      else
        throw MUK_EXCEPTION_SIMPLE("There are no paths that satisfy the filters");
      mpModels->pVisModel->render();
    }

    /** gets the bestPathIndex from the model and logs it 
    */
    void SelectionController::evaluatePaths()
    {
      const auto bestPathIndex = myModel->evaluatePaths();
      if (bestPathIndex >= 0)
      {
        LOG_LINE << "Best Path of the filtered Paths is path Nr. " << bestPathIndex;
        highlightViaSpinBox(bestPathIndex);
      }
    }

    /** if the button is clicked it is checked in which state the tab is right now and depending on that the new state is chosen
    */
    void SelectionController::advancedOptionsWindow()
    {
      auto myTabSelection = mpMainWindow->mpTabSelection;
      if (!mTabIsWindow && !mTabIsLarge) // when the tab was small it will be set to windowed mode
      {
        mTabIsWindow = true;
        // if called the first time the advanced calculations will be enabled and recalculation is needed
        mTabNeedsUpdate = true;
        updateSelectionTab(); // always update the displayed values if changes in small happend
        auto tabSelectionWindow = new TabSelectionWindow(); // the window
        tabSelectionWindow->showMaximized(); 
        tabSelectionWindow->setWindowTitle("Advanced Selection");
        tabSelectionWindow->setObjectName("TabWindow");
        tabSelectionWindow->setMinimumSize(1300, myTabSelection->size().height());
        tabSelectionWindow->setCentralWidget(myTabSelection);
        tabSelectionWindow->connect(tabSelectionWindow, &QMainWindow::destroyed, this, &SelectionController::windowClosed);
        // reparent the selection tab 
        myTabSelection->setParent(tabSelectionWindow);
        // need to call show again after reparenting
        myTabSelection->show();
        // get the right layout
        myTabSelection->setLayoutLarge(true);
        // make sure that the count of obstacles and components is correct
        myTabSelection->updateComponentCount(mCTFileLoaded, false);
        updateTabSelectionObstacles();
        // change the text on the buttons
        myTabSelection->toggleAdvancedOptionsWindow(true);
        // since the selection tab is now a window (and now longer in the tab container) change the current index to planning
        mpMainWindow->mpTabContainer->setCurrentIndex(MuknoPlannerMainWindow::enTabPlanning);
      }
      else if (mTabIsLarge) // when it was large change it to window, no need to recalculate
      {
        mTabIsLarge = false;
        mpMainWindow->mpTabContainer->setMinimumWidth(0);
        myTabSelection->toggleAdvancedOptionsEnlarge(false);
        mTabIsWindow = true;
        auto tabSelectionWindow = new TabSelectionWindow();
        tabSelectionWindow->showMaximized();
        tabSelectionWindow->setWindowTitle("Advanced Selection");
        tabSelectionWindow->setObjectName("TabWindow");
        tabSelectionWindow->setMinimumSize(1300, myTabSelection->size().height());
        tabSelectionWindow->setCentralWidget(myTabSelection);
        tabSelectionWindow->connect(tabSelectionWindow, &QMainWindow::destroyed, this, &SelectionController::windowClosed);
        myTabSelection->setParent(tabSelectionWindow);
        myTabSelection->show();
        mpMainWindow->mpTabContainer->setCurrentIndex(MuknoPlannerMainWindow::enTabPlanning);
        myTabSelection->toggleAdvancedOptionsWindow(true);
      }
      else // when it was windowed already set it to small
      {
        mTabIsWindow = false;
        auto window = dynamic_cast<TabSelectionWindow*>(myTabSelection->parent());
        // close the window and put the tab back in the tabcontainer and set the current index to it
        window->close();
        mpMainWindow->mpTabContainer->insertTab(MuknoPlannerMainWindow::enTabSelection, myTabSelection, "Selection");
        mpMainWindow->mpTabContainer->setCurrentIndex(MuknoPlannerMainWindow::enTabSelection);
        myTabSelection->setLayoutLarge(false);
        myTabSelection->toggleAdvancedOptionsWindow(false);
      }
    }

    /** if the button is clicked it is checked in which state the tab is right now and depending on that the new state is chosen
        works almost the same as advancedOptionsWindow
    */
    void SelectionController::advancedOptionsEnlarge()
    {
      auto myTabSelection = mpMainWindow->mpTabSelection;
      if (!mTabIsLarge && !mTabIsWindow) // = TabIsSmall
      {
        mTabIsLarge = true;
        mpMainWindow->mpTabContainer->setMinimumWidth(1300);
        myTabSelection->setLayoutLarge(true);
        myTabSelection->updateComponentCount(mCTFileLoaded, false);
        updateTabSelectionObstacles();
        myTabSelection->toggleAdvancedOptionsEnlarge(true);
        mTabNeedsUpdate = true;
        updateSelectionTab();
      }
      else if (mTabIsWindow) // = TabIsWindow
      {
        mTabIsWindow = false;
        auto window = dynamic_cast<TabSelectionWindow*>(myTabSelection->parent());
        window->close();
        myTabSelection->toggleAdvancedOptionsWindow(false);
        mpMainWindow->mpTabContainer->insertTab(MuknoPlannerMainWindow::enTabSelection, myTabSelection, "Selection");
        mpMainWindow->mpTabContainer->setCurrentIndex(MuknoPlannerMainWindow::enTabSelection);
        mTabIsLarge = true;
        mpMainWindow->mpTabContainer->setMinimumWidth(1300);
        myTabSelection->toggleAdvancedOptionsEnlarge(true);
      }
      else // = TabIsLarge
      {
        mTabIsLarge = false;
        mpMainWindow->mpTabContainer->setMinimumWidth(0);
        myTabSelection->setLayoutLarge(false);
        myTabSelection->toggleAdvancedOptionsEnlarge(false);
      }
    }

    /** Called when the TabSelection Window is closed or the MuknoPlanner is about to be closed
    */
    void SelectionController::windowClosed()
    {
      if(mTabIsWindow)
        advancedOptionsWindow();
    }

    /** \perform a quick analysis of the active path and show the results in customized widget
    */
    void SelectionController::quickPathAnalysis()
    {
      if (mActiveKey.empty())
        return;

      auto* pScene = mpModels->pAppModel->getScene().get();
      auto& coll = pScene->getPathCollection(mActiveKey);
      const size_t N = coll.getPaths().size();
      if (N == 0)
        return;

      const size_t idx = mpMainWindow->mpTabSelection->getSinglePathSelection();
      if (idx<0 || idx >= N)
      {
        throw MUK_EXCEPTION_SIMPLE("active path index is invalid");
      }

      //auto pVisPath = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey)->getMukPath(idx); // take the data from the visualization!
      //MukPath path = pVisPath->asMukPath();
      //// compute distances and curvatures along the path
      //auto distances = computeDistances(*pScene->getCollisionDetector(), path.getStates(), coll.getProblemDefinition()->getRadius());
      //auto curvatures = computeCurvatures(path.getStates());
      const double minDist = coll.getProblemDefinition()->getSafetyDist();/*
      //const double maxKappa = coll.getProblemDefinition()->getKappa();*/

      //// color the path red at points where the path violates any constraints
      //std::vector<Vec3d> colors;
      //colors.reserve(distances.size());
      //{
      //  std::transform(distances.begin(), distances.end(), curvatures.begin(), std::back_inserter(colors), [&](double dist, double kappa)
      //  {
      //    if (dist < minDist || kappa > maxKappa)
      //      return Vec3d(Colors::Red[0], Colors::Red[1], Colors::Red[2]);
      //    else
      //      return Vec3d(Colors::Green[0], Colors::Green[1], Colors::Green[2]);
      //  });
      //}
      //pVisPath->setColors(colors);


      // create the plot
      mpPlot = std::make_unique<QuickPathAnalysisWindow>();
      {
        // adjust
        addPath(idx);
        mpPlot->setSafetyDistance(minDist);
        // old stuff
        /*mpPlot->setCurvatures(curvatures);
        mpPlot->setKappa(maxKappa);*/
        // replot
        mpPlot->compute();
        mpPlot->open();
        connect(mpPlot.get(), &QuickPathAnalysisWindow::plotDoubleClicked, this, [=](QMouseEvent * event) {plotDoubleClicked(event, mpPlot.get()) ; });
        connect(mpPlot.get(), &QuickPathAnalysisWindow::pathRequestedClicked, this, &SelectionController::addPath);
      }
    }

    /**
    */
    void SelectionController::addPath(int idx)
    {
      if (mActiveKey.empty())
        return;

      auto* pScene = mpModels->pAppModel->getScene().get();
      auto& coll = pScene->getPathCollection(mActiveKey);
      const size_t N = coll.getPaths().size();
      if (N == 0)
        return;

      if (idx<0 || idx >= N)
      {
        throw MUK_EXCEPTION_SIMPLE("active path index is invalid");
      }
      LOG_LINE << "Adding idx " << idx;
      auto pVisPath = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey)->getMukPath(idx); // take the data from the visualization!
      MukPath path = pVisPath->asMukPath();
      // compute disatnces along the path
      auto distances = computeDistances(*pScene->getCollisionDetector(), path.getStates(), coll.getProblemDefinition()->getRadius());
      const double minDist = coll.getProblemDefinition()->getSafetyDist();

      // compute closest point to each risk structure
      std::vector<QuickPathAnalysisWindow::ObstaclePeak> peaks;
      {
        auto keys = pScene->getObstacleKeys();
        CollisionDetectionHandler handler;
        handler.initialize(pScene->getCollisionDetector().get());
        for (const auto& key : keys)
        {
          if (!pScene->getObstacle(key)->getActive())
            continue;
          handler.setupForObstacle(key);
          auto pair = handler.distance(path, coll.getProblemDefinition()->getRadius());
          peaks.push_back(std::make_tuple(key, pair.first, pair.second));
        }
      }

      mpPlot->addDistances(distances);
      mpPlot->addPeaks(peaks);
      mpPlot->compute();
    }

    /** when a double click in the pathAnalysis window is registered the closest state to the mouse is determined
        and the nearest neighbor to that state is shown in the planner (and colered according to the distance)

        TODO: fix this properly, as currently more than one graph could be displayed.
    */
    void SelectionController::plotDoubleClicked(QMouseEvent * mouseEvent, QuickPathAnalysisWindow * window)
    {
      if (window->getPlot()->graphCount() == 0)
        return;
      auto* graph = window->getPlot()->graph(0);
      auto* data = graph->data();
      auto xAxis = data->values();
      auto yAxis = data->keys();
      auto keyAxis = graph->keyAxis();
      auto minDif = std::numeric_limits<double>::infinity();
      auto pathState = -1;
      // for every state of the x axis the distance to the mouse position is calculated
      for (auto key : xAxis)
      {
        auto tempDif = abs(keyAxis->coordToPixel(key.key) - mouseEvent->pos().x());
        if (tempDif < minDif)
        {
          pathState = key.key;
          minDif = tempDif;
        }
      }
      auto pathIndx = mpMainWindow->mpTabSelection->getSinglePathSelection();
      auto pVisPath = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey)->getMukPath(pathIndx);
      auto stateCoords = pVisPath->asMukPath().getStates()[pathState].coords;
      // sets the cursor position for the method show closest point to work
      mpControls->mpVisControl->setCursorPosition(stateCoords);
      mpControls->mpInteract->showClosestPoint();
    }

    /* when ct files are loaded all calculations and displays working with the number of components are updated
    **/
    void SelectionController::ctFileLoaded()
    {
      if (!mCTFileLoaded)
      {
        mCTFileLoaded = true;
        myModel->ctFileLoaded();
        mpMainWindow->mpTabSelection->updateComponentCount(mCTFileLoaded, !(mTabIsLarge || mTabIsWindow));
        mTabNeedsUpdate = true;
        updateSelectionTab();
      }
    }

    /** when the active property in the property window changes (only obstacle use that right now) this method
        counts the active obstacle and sends them to the calculations and the display for them to only work on the active obstacles
    */
    void SelectionController::updateActiveObstacles()
    {
      auto* pScene = mpModels->pAppModel->getScene().get();
      auto keys = pScene->getObstacleKeys();
      std::vector<std::string> temp;
      for (auto key : keys)
      {
        if (pScene->getObstacle(key)->getActive())
          temp.push_back(key);
      }
      if (temp.size() > maxObstacleCount)
        throw MUK_EXCEPTION_SIMPLE("The Selection Tab can show a maximum of 12 active obstacles at the same time. Deactivate some other Obstacles before you add more");
      mActiveObstacles = temp;
      obstacleCount = mActiveObstacles.size();

      myModel->setActiveObstacles(mActiveObstacles);
      obstacleWeightingChanged();
    }

    /** updates the Obstacles in the Selection Tab, Colors in the ParameterNet and which Obstacles are shown aswell as Names of the Obstacles
    */
    void SelectionController::updateTabSelectionObstacles()
    {
      std::vector<QColor> colors;
      auto mpVisScene = mpModels->pVisModel->getVisScene();
      for (auto key : mActiveObstacles)
      {
        colors.push_back(QColor(mpVisScene->getObstacle(key)->getDefaultColor().x() * 255,
          mpVisScene->getObstacle(key)->getDefaultColor().y() * 255,
          mpVisScene->getObstacle(key)->getDefaultColor().z() * 255));
      }
      mpMainWindow->mpTabSelection->updateObstacleCount(!(mTabIsLarge || mTabIsWindow), mActiveObstacles, colors);
    }

    /** updates the model with te values from the tab
    */
    void SelectionController::componentWeightingChanged()
    {
      if (mActiveKey.empty() || mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      else
      {
        auto wComp = mpMainWindow->mpTabSelection->getComponentWeights();
        myModel->setComponentWeights(wComp);
      }
    }

    /** updates the model with the value from the tab
    */
    void SelectionController::obstacleWeightingChanged()
    {
      if (mActiveKey.empty() || mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        return;
      }
      else
      {
        auto wObs = mpMainWindow->mpTabSelection->getObstacleWeights();
        wObs.resize(obstacleCount); // maybe needs to be cut abit when there where more obstacles before because one got removed
        myModel->setObstacleWeights(wObs);
      }
    }

    /** When a Filter-Slider is changed this computes the filters for the Model and sends the Values for the Labels back to Tab Selection
    */
    void SelectionController::componentFilterChanged(int ind)
    { 
      if (!mActiveKey.empty() && !mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        double maxFilter = 1000.0; // the filters have 1000 steps
        auto fComp = mpMainWindow->mpTabSelection->getComponentFilter();
        std::vector<double> filterValue;
        std::vector<double> filterMin;
        std::vector<double> filterMax;
        // fill the vectors with the worst and best of each category to determine the min and max of the filter and what each step amounts to
        std::vector<double> theWorst{ 0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
        std::vector<double> theBest{ std::numeric_limits<double>::infinity(), 0, 0, 0, 0, 0 };
        auto oldValues = myModel->getComponentFilter();
        auto tempBest = myModel->getCurrentBest();
        theWorst[0] = myModel->getDistances()[myModel->getDistanceOrder().back()];
        theBest[0] = tempBest.distance.second;
        theWorst[1] = myModel->getCurvatures()[myModel->getCurvatureOrder().back()];
        theBest[1] = tempBest.curvature.second;
        theWorst[2] = myModel->getGoalAngles()[myModel->getAngleOrder().back()];
        theBest[2] = tempBest.angle.second;
        theWorst[3] = myModel->getLengths()[myModel->getLengthOrder().back()];
        theBest[3] = tempBest.length.second;
        if (mCTFileLoaded) {
          theWorst[4] = myModel->getBoneThickness()[myModel->getBoneOrder().back()];
          theBest[4] = tempBest.boneThickness.second;
          theWorst[5] = myModel->getAirHoles()[myModel->getAirholesOrder().back()];
          theBest[5] = tempBest.airhole.second;
        }
        for (size_t i(0); i < fComp.size(); i++)
        { // calculates to which value the current filter setting corresponds to
          auto tempValue = theBest[i] + (maxFilter - (double)fComp[i]) * ((theWorst[i] - theBest[i]) / maxFilter);
          // i == ind checks if the changed index (ind) equals the current index (i) if so the sliderValue is just set
          // the sliderValue is set when the filterValue was never set before (when oldVal was initVal) too
          if ((int)i == ind || oldValues[i] == componentFiltersInitValue)
          {
            if (i == 0) // need another calculation for distance because bigger is better (for everything else smaller is better)
              filterValue.push_back(theWorst[i] + (double)fComp[i] * ((theBest[i] - theWorst[i]) / maxFilter));
            else
              filterValue.push_back(tempValue);
          }
          else // if not then the new filterValue of ind might influence all other filters so their slider positions need to be recalculated
          {
            filterValue.push_back(oldValues[i]); // take the unchanged old value for the filter
            if (myModel->roundDouble(tempValue, 5) != myModel->roundDouble(oldValues[i], 5)) // when the sliderValue changed because of ind
            { // then reverse calculate the sliderValue out of the filterValue
              if (i == 0)
                fComp[i] = round((oldValues[i] - theWorst[i]) * maxFilter / (theBest[i] - theWorst[i]));
              else
                fComp[i] = round(-((oldValues[i] - theBest[i]) * maxFilter / (theWorst[i] - theBest[i]) - maxFilter));
            }
          }
          filterMin.push_back(theWorst[i]);
          filterMax.push_back(theBest[i]);
        }
        myModel->setComponentFilter(filterValue);
        mpMainWindow->mpTabSelection->updateComponentFilter(fComp, filterValue, filterMin, filterMax);
      }
      else
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
    }

    /** \brief When a Filter-Slider is changed this computes the filters for the Model and sends the values for the labels back to Selection Tab
    */
    void SelectionController::obstacleFilterChanged(int ind)
    {
      // behaves like the distance component of componentFilterChanged, just for every obstacle
      if ( ! mActiveKey.empty() 
        && ! mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        mfilteredPaths = myModel->getFilteredPaths();
        double maxFilter = 1000.0;
        auto fObs = mpMainWindow->mpTabSelection->getObstacleFilter();
        fObs.resize(obstacleCount);
        std::vector<double> filterValue;
        std::vector<double> filterMin;
        std::vector<double> filterMax;
        double worstDistance = 0;
        double bestDistance = std::numeric_limits<double>::infinity();
        auto   oldValues    = myModel->getObstacleFilter();
        auto minDistToObst = myModel->getMinDistToEachObstacle();
        if (minDistToObst.empty())
          return;
        for (size_t i(0); i < obstacleCount; ++i)
        {
          std::vector<double> obstacleDistancesMax;
          std::vector<double> obstacleDistancesMin;
          for (size_t j(0); j < minDistToObst.size(); j++)
          {
            // get the best and worst distance to every obstacle
            if (std::find(mfilteredPaths.begin(), mfilteredPaths.end(), j) != mfilteredPaths.end())
              obstacleDistancesMax.push_back(minDistToObst[j][i]);
            obstacleDistancesMin.push_back(minDistToObst[j][i]);
          }
          worstDistance = *std::min_element(obstacleDistancesMin.begin(), obstacleDistancesMin.end());
          bestDistance = *std::max_element(obstacleDistancesMax.begin(), obstacleDistancesMax.end());
          double tempValue = worstDistance + (double)fObs[i] * ((bestDistance - worstDistance) / maxFilter);
          if ((int)i == ind || oldValues[i] == obstacleFiltersInitValue)
          {
            filterValue.push_back(tempValue);
          }
          else
          {
            if (i >= oldValues.size())
              filterValue.push_back(tempValue);
            else
            {
              filterValue.push_back(oldValues[i]);
              if (myModel->roundDouble(tempValue, 5) != myModel->roundDouble(oldValues[i], 5))
              {
                fObs[i] = (size_t)round((oldValues[i] - worstDistance) * maxFilter / (bestDistance - worstDistance));
              }
            }
          }
          filterMin.push_back(worstDistance);
          filterMax.push_back(bestDistance);
        }
        myModel->setObstacleFilter(filterValue);
        mpMainWindow->mpTabSelection->updateObstacleFilter(fObs, filterValue, filterMin, filterMax);
      }
      else
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
    }

    /** Resets all Changes in the Model and TabSelection when the planning Changed or the Scene was cleared
    */
    void SelectionController::resetTabSelection()
    {
      mPositionInPath = 0;
      mfilteredPaths.clear();
      auto accessCanals = myModel->selectedIndices();
      // Unsets all AccessCanals that are not set as an obstacle (or already unset)
      for (size_t i(0); i < accessCanals.size(); i++)
      {
        if (!mSetAsObstacle[i] && accessCanals[i] != -1)
        {
          myModel->setAccessCanal(i, accessCanals[i]);
          mpMainWindow->mpTabSelection->setAccessCanalIndex(i, -1);
          mpMainWindow->mpTabSelection->updateCutOffDistance(i, 1);
        }
      }
      mpMainWindow->mpTabSelection->setSinglePathSelection(-1);
      mpMainWindow->mpTabSelection->checkAccessCanals(myModel->selectedIndices());
      mPathIsColored = false;
      mpModels->pVisModel->render();
      updateActiveObstacles();
      mpMainWindow->mpTabSelection->resetTabSelection();
      myModel->setComponentWeights(std::vector<double>(componentCount, componentWeightsInitValue));
      myModel->setComponentFilter(std::vector<double>(componentCount, componentFiltersInitValue));
      myModel->setObstacleWeights(std::vector<double>(obstacleCount, obstacleWeightsInitValue));
      myModel->setObstacleFilter(std::vector<double>(obstacleCount, obstacleFiltersInitValue));
      mTabNeedsUpdate = true;
      updateSelectionTab();
    }

    /** resets all calculations regarding filters and weights and recalculates the model
    */
    void SelectionController::recalculateModel()
    {
      LOG_LINE << "Recalculating the Model";
      mfilteredPaths.clear();
      updateActiveObstacles();
      mpMainWindow->mpTabSelection->resetTabSelection();
      myModel->setComponentWeights(std::vector<double>(componentCount, componentWeightsInitValue));
      myModel->setComponentFilter(std::vector<double>(componentCount, componentFiltersInitValue));
      myModel->setObstacleWeights(std::vector<double>(obstacleCount, obstacleWeightsInitValue));
      myModel->setObstacleFilter(std::vector<double>(obstacleCount, obstacleFiltersInitValue));
      mTabNeedsUpdate = true;
      updateSelectionTab();
    }

    /** activates the CT-Overlay or deactivates it depending on its activity
    */
    void SelectionController::toggleCTOverlay()
    {
      LOG_LINE << __FUNCTION__;
      if (!mCTFileLoaded)
      {
        throw MUK_EXCEPTION_SIMPLE("no CT-File loaded yet");
      }
      if ( ! mpModels->pVisModel->hasAbstractObject(OverlayImageName))
      {
        LOG_LINE << "add Overlay";
        auto pObj = std::make_shared<VisImageOverlay>(OverlayImageName);
        auto pImage = mpModels->pWorldVisModel->getCtImage();
        pObj->setImage(pImage);
        mpModels->pVisModel->addAbstractObject(pObj);
        mpControls->mpPropControl->reloadProperty();
        pObj->removeProperty("Color");
        mOverlayIsActive = true;
      }
      else
      {
        LOG_LINE << "remove Overlay";
        auto* pObj = dynamic_cast<VisImageOverlay*>(mpModels->pVisModel->getVisScene()->getObject(OverlayImageName).get());
        pObj->setVisibility(false);
        mpModels->pVisModel->deleteAbstractObject(OverlayImageName);
        mpControls->mpPropControl->reloadProperty();
        mOverlayIsActive = false;
        mPositionInPath = 0;
        mpMainWindow->mpTabSelection->updateDisplayState(mPositionInPath);
      }
      mpMainWindow->mpTabSelection->toggleCTOverlay(mOverlayIsActive);
      displayCTSlice();
    }

    /** when scrolling in the tab the position in path of the CT-Overlay is changed
    */
    void SelectionController::scrollCTOverlay(bool forward)
    {
      if (mOverlayIsActive)
      {
        const auto* path = mpModels->pSelectionModel->getLoadedPath();
        if (path == nullptr || path->getStates().empty())
          return;
        // won't increase the positionInPath when the position is at the end of the path
        if (forward)
          ++mPositionInPath;
        else
          --mPositionInPath;
        // assert position is within size
        mPositionInPath = std::max(0, std::min((int)path->getStates().size() - 1, mPositionInPath));
        mpMainWindow->mpTabSelection->updateDisplayState(mPositionInPath, path->getStates().size()-1);
        displayCTSlice();
      }
    }

    /** when scrolling in the spinbox the position in path of the CT-Overlay is changed
    */
    void SelectionController::ctStateChanged(size_t i)
    {
      if (mOverlayIsActive)
      {
        const auto* path = mpModels->pSelectionModel->getLoadedPath();
        mPositionInPath = std::min(i, path->getStates().size()-1);
        if(i > path->getStates().size()-1)
          // set the max of the spinbox only when the requested state is out of the limits to make it possible to set a very high number and let 
          // the spinbox snap to the max automaticly
          mpMainWindow->mpTabSelection->updateDisplayState(mPositionInPath, path->getStates().size()-1);
        else
          mpMainWindow->mpTabSelection->updateDisplayState(mPositionInPath, std::numeric_limits<int>::max());
        displayCTSlice();
      }
      else
      {
        mpMainWindow->mpTabSelection->updateDisplayState(mPositionInPath);
        throw MUK_EXCEPTION_SIMPLE("CT-Overlay is not active");
      }
    }

    /** \brief updates the position of the CT-Overlay when activated or when the active path was changed
      
      updating the Position when the CT-Overlay was toggled 
      updating the Position when the active path was changed
    */
    void SelectionController::displayCTSlice()
    {
      if (mOverlayIsActive)
      {
        auto* pScene = mpModels->pAppModel->getScene().get();
        auto& coll = pScene->getPathCollection(mActiveKey);
        const size_t N = coll.getPaths().size();
        if (N == 0)
          return;
        const size_t idx = mpMainWindow->mpTabSelection->getSinglePathSelection();
        if (idx<0 || idx >= N)
        {
          toggleCTOverlay();
          throw MUK_EXCEPTION_SIMPLE("active path index is invalid");
        }
        auto pVisPath = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey)->getMukPath(idx); // take the data from the visualization!
        MukPath path = pVisPath->asMukPath();
        LOG_LINE << "position " << mPositionInPath << " of " << path.getStates().size() - 1;
        auto* pObj = dynamic_cast<VisImageOverlay*>(mpModels->pVisModel->getVisScene()->getObject(OverlayImageName).get());
        pObj->setNormal(path.getStates()[mPositionInPath].tangent);
        pObj->setPosition(path.getStates()[mPositionInPath].coords);
      }
      mpModels->pVisModel->render();
    }

    /** when the button is pushed the corresponding canal is set to the ind displayed in the single selection spinbox
    */
    void SelectionController::selectAccessCanal(size_t canalIdx)
    {
      if (!mActiveKey.empty() && !mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        auto pathIdx = mpMainWindow->mpTabSelection->getSinglePathSelection();
        if (pathIdx == -1)
          throw MUK_EXCEPTION_SIMPLE("the invalid path '-1' can't be set as an Access Canal");
        auto accessCanals = myModel->selectedIndices();
        // sets the canal in the model, but when it already was at that pathIdx it unsets it instead
        myModel->setAccessCanal(canalIdx, pathIdx);
        // when it was unset the pathIdx is set to -1 for the other methods
        if (myModel->selectedIndices()[canalIdx] == -1)
        {
          markPathAsSelected(-1, accessCanals[canalIdx]);
          pathIdx = -1;
        } // if it was unset before
        else if (accessCanals.empty() || (int)accessCanals[canalIdx] == -1 || accessCanals.size() <= canalIdx)
          markPathAsSelected(pathIdx, -1);
        else // if it was set as another path before
          markPathAsSelected(pathIdx, accessCanals[canalIdx]);
        mpMainWindow->mpTabSelection->setAccessCanalIndex(canalIdx, pathIdx);
        mpMainWindow->mpTabSelection->checkAccessCanals(myModel->selectedIndices());
        mpMainWindow->mpTabSelection->updateCutOffDistance(canalIdx, 1);
        mpModels->pVisModel->render();
        myModel->filterPaths();
        componentFilterChanged(-1);
        obstacleFilterChanged(-1);
      }
      else
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
    }

    /** takes the cutOffDistance from tab selection and determines all points that have a lower distance to the endPoint and marks them
        for removal when setting the obstacle
    */
    std::vector<bool> SelectionController::cutOffDistanceChanged(size_t i, double cutOffDist)
    {
      if (!mActiveKey.empty() && !mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        auto accessCanals = myModel->selectedIndices();
        auto minCutOffDistance = mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getProblemDefinition()->getGoalThreshold();
        if (accessCanals.empty() || accessCanals[i] == -1 || accessCanals.size() <= i)
        {
          mpMainWindow->mpTabSelection->updateCutOffDistance(i, minCutOffDistance);
          throw MUK_EXCEPTION_SIMPLE("The AccessCanal was not set yet");
        }
        else
        { // when the cutOffDist is smaller then the minCutOffDist set in the property Widget
          if (cutOffDist < minCutOffDistance)
            cutOffDist = minCutOffDistance;
          // make sure the path is highlighted
          highlightSinglePath(accessCanals[i]);
          auto currentPath = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey)->getMukPath(accessCanals[i]);
          auto pathStates = currentPath->asMukPath().getStates();
          // the GoalPoint, assuming there is only one
          auto startPoint = mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getProblemDefinition()->getStartStates()[0];
          auto cutOffState = myModel->determineCutOffState(pathStates, startPoint, cutOffDist);
          std::vector<bool> statesToMark; // looks which states fulfill the criteria and which not
          for (size_t i(0); i < pathStates.size(); i++)
          {
            if (i <= cutOffState)
              statesToMark.push_back(true);
            else
              statesToMark.push_back(false);
          }
          markStatesInPath(currentPath, statesToMark);
          mpMainWindow->mpTabSelection->updateCutOffDistance(i, cutOffDist);
          return statesToMark; // returns the boolean vector for setCanalAsObstacle
        }
      }
      else
      {
        mpMainWindow->mpTabSelection->updateCutOffDistance(i, 1.0);
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
    }

    /** sets an AccessCanal as an obstacle but without the marked states
    */
    void SelectionController::setCanalAsObstacle(size_t i)
    { // a booleanVector for the state of every canal
      if (mSetAsObstacle[i]) // was set as an Obstacle before -> remove the obstacle
      {
        LOG_LINE << "Obstacle AccessCanal " << i + 1<< " was removed";
        mpModels->pVisModel->deleteObstacle("AccessCanal " + std::to_string(i + 1));
        mpModels->pAppModel->getScene()->deleteObstacle("AccessCanal " + std::to_string(i + 1));
        mpModels->pAppModel->getScene()->getCollisionDetector()->rebuild();
        mpControls->mpPropControl->initSceneWidget();
        // shows the path again if it can be fetched
        if (!mActiveKey.empty())
        {
          if (!mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
          {
            auto accessCanals = myModel->selectedIndices();
            if (mSetAsObstacle[i] && !accessCanals.empty() && accessCanals[i] != -1 && accessCanals.size() > i)
            {
              auto currentPath = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey)->getMukPath(accessCanals[i]);
              currentPath->setVisibility(true);
            }
          }
        }
      }
      else if (!mActiveKey.empty() && !mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        auto accessCanals = myModel->selectedIndices();
        if (accessCanals.empty() || accessCanals[i] == -1 || accessCanals.size() <= i)
        {
          throw MUK_EXCEPTION_SIMPLE("The AccessCanal was not set yet");
        }
        else
        {
          if (mActiveObstacles.size() >= maxObstacleCount)
            throw MUK_EXCEPTION_SIMPLE("The Selection Tab can show a maximum of 12 active obstacles at the same time. Deactivate some other Obstacles before you add more");
          LOG_LINE << "Added Obstacle AccessCanal " << i + 1;
          auto cutOffStates = cutOffDistanceChanged(i, mpMainWindow->mpTabSelection->getCutOffDistance(i));
          auto currentPath = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey)->getMukPath(accessCanals[i]);
          auto tempPath = currentPath->asMukPath();
          auto tempPathStates = tempPath.getStates();
          auto newPathStates = std::vector<MukState>();
          // makes a new mukPath with the same states until the cutOffPoint
          for (size_t i(0); i < tempPathStates.size(); i++)
          {
            if (!cutOffStates[i])
              newPathStates.push_back(tempPathStates[i]);
          }
          tempPath.setStates(newPathStates);
          auto cutPath = VisMukPath(tempPath);
          cutPath.setData(tempPath);

          auto cutPathData = cutPath.getData();
          // used the same process as the tube topology for the paths
          auto cleanFilter = make_vtk<vtkCleanPolyData>();
          cleanFilter->SetInputData(cutPathData);
          cleanFilter->Update();

          auto data = cleanFilter->GetOutput();
          DefVtk(vtkTubeFilter, tube);
          tube->SetInputData(data);
          tube->SetNumberOfSides(20);
          tube->SetRadius(tempPath.getRadius());
          tube->Update();

          auto pObj = std::make_shared<MukObstacle>();
          pObj->setName("AccessCanal " + std::to_string(i + 1));
          pObj->setData(tube->GetOutput());
          pObj->setActive(true);
          currentPath->setVisibility(false);
          mpModels->pAppModel->getScene()->insertObstacle(pObj);
          mpModels->pAppModel->getScene()->getCollisionDetector()->rebuild();
          mpModels->pVisModel->addObstacle(pObj->getName());
          mpControls->mpPropControl->initSceneWidget();
        }
      }
      else
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
      // no matter if set or not set, if no exception was thrown, the corresponding boolean is reversed and the text on the button is set accordingly
      mSetAsObstacle[i] = !mSetAsObstacle[i];
      mpMainWindow->mpTabSelection->toggleAsObstacle(mSetAsObstacle[i], i);

      mpModels->pVisModel->render();

      recalculateModel();
    }

    /** automaticaly fills the unset canals with the ones the most apart (how it works is in the model)
        and makes sure each of them is visible and marks the cutOffState of each path
    */
    void SelectionController::autoFillCanals()
    {
      if (!mActiveKey.empty() && !mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getPaths().empty())
      {
        const auto pVisColl = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey);
        auto startPoint = mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getProblemDefinition()->getStartStates()[0];
        auto goalThreshold = mpModels->pAppModel->getScene()->getPathCollection(mActiveKey).getProblemDefinition()->getGoalThreshold();
        auto theSelection = myModel->determineBestCanals(startPoint);
        for (size_t i(0); i < accessCanalCount; i++)
        {
          markPathAsSelected(theSelection[i], -1);
          auto currentStates = pVisColl->getMukPath(theSelection[i])->asMukPath().getStates();
          auto cutOffState = myModel->determineCutOffState(currentStates, startPoint, goalThreshold);
          auto statesToMark = std::vector<bool>(currentStates.size(), false);
          statesToMark[cutOffState] = true;
          markStatesInPath(pVisColl->getMukPath(theSelection[i]), statesToMark);
          mpMainWindow->mpTabSelection->setAccessCanalIndex(i, theSelection[i]);
          mpMainWindow->mpTabSelection->updateCutOffDistance(i, 1);
        }
        mpMainWindow->mpTabSelection->checkAccessCanals(myModel->selectedIndices());
        mpModels->pVisModel->render();
        updateSelectionTab();
        LOG_LINE << "AccessCanals filled with the ones the most apart";
      }
      else
      {
        throw MUK_EXCEPTION_SIMPLE("The Path Collection is empty");
      }
    }

    /** Marks the true Idx in statesToMark in the path blue
    */
    void SelectionController::markStatesInPath(std::shared_ptr<VisMukPath> path, std::vector<bool> statesToMark)
    {
      std::vector<Vec3d> colors(path->asMukPath().getStates().size(), Vec3d(0, 1, 0));
      for (size_t i(0); i < colors.size(); i++)
      {
        if (statesToMark[i])
          colors[i] = Vec3d(0, 0, 1);
      }
      path->setColors(colors);
      path->reloadTopology();
      mpModels->pVisModel->render();
    }

    /** \brief
    */
    std::vector<std::string> SelectionController::getParetoParams() const
    {
      std::vector<std::string> params;
      params.push_back(Invalid_Choice);
      params.push_back(Path_Length);
      params.push_back(Min_Distance);
      for (const auto& obs : mActiveObstacles)
        params.push_back(obs);
      return params;
    }

    /** called when clicked on the show paretoFrontWindow button in tabselection. Opens a new Window with the same widgets
    */
    void SelectionController::showParetoFrontWindow()
    {
      mpParetoWindow.reset(new TabSelectionWindow()); // the window
      //paretoWindow->showMaximized();
      mpParetoWindow->show();
      mpParetoWindow->setWindowTitle("ParetoFront");
      mpParetoWindow->setObjectName("ParetoFrontWindow");
      mpParetoWindow->setMinimumSize(1275, 850);

      mpParetoWidget = new ParetoWidget(nullptr);
      auto params = getParetoParams();
      mpParetoWidget->setParameterList(params);
      mpParetoWindow->setCentralWidget(mpParetoWidget);
      mpMainWindow->mpTabSelection->paretoStatus(true);
      //mpParetoWidget->connect(mpParetoWindow.get(), &QMainWindow::destroyed, [=] { mpMainWindow->mpTabSelection->paretoStatus(false); });

      connect(mpParetoWindow.get(), &QMainWindow::destroyed, [=] { mpMainWindow->mpTabSelection->paretoStatus(false); });
      connect(mpParetoWindow.get(), &QMainWindow::destroyed, this, &SelectionController::exitParetoFrontWindow);
      connect(mpParetoWidget, &ParetoWidget::clickedPath, mpMainWindow->mpTabSelection, &TabSelection::setSinglePathSelection); //[=] (size_t ind) { mpMainWindow->mpTabSelection->mpSelectedPath->setValue(int(ind)); });
      connect(mpParetoWidget, &ParetoWidget::parameterChosen, this, &SelectionController::paretoParameterChosen); //[=] (size_t ind) { mpMainWindow->mpTabSelection->mpSelectedPath->setValue(int(ind)); });
      //comboBox->connect(comboBox, SELECT<const QString&>::OVERLOAD_OF(&QComboBox::currentIndexChanged), this, [&](const QString& str) { paramChosen(false, str); emit this->parameterChosen(false, str);});
      //connect(pTabSelection, &SelectionController::parameterChosen, this, &SelectionController::paretoParameterChosen);
    }

    /** called when clicked on the exit button in the paretoFrontWindow. Closes the Window
    */
    void SelectionController::exitParetoFrontWindow()
    {
      mpParetoWindow->close();
      mpParetoWidget = nullptr;
    }

    /** called when a parameter is chosen in the paretoFrontWindow.
      sets the parameter for the ParetoFrontGraphic and calls the ParetoFront-Widget of the Window
    */
    void SelectionController::paretoParameterChosen(bool isParam1, const QString& parameterName) const
    {
      const auto &filteredPaths = myModel->getFilteredPaths();
      const std::vector<double>* parameterList  = nullptr;
      const std::vector<size_t>* parameterOrder = nullptr;
      const auto paramName = parameterName.toStdString();
      if (paramName == Invalid_Choice)
      {
        parameterList = nullptr;
        parameterOrder = nullptr;
      }
      else if (paramName == Path_Length)
      {
        parameterList  = &myModel->getLengths();
        parameterOrder = &myModel->getLengthOrder();
      }
      else if (paramName == Min_Distance)
      {
        parameterList  = &myModel->getDistances();
        parameterOrder = &myModel->getDistanceOrder();
      }
      else
      {
        bool found = false;
        size_t obstacleIndex;
        for (size_t i(0); i < mActiveObstacles.size(); ++i)
        {
          if (paramName == mActiveObstacles[i])
          {
            found = true;
            obstacleIndex = i;
            break;
          }
        }
        if (found)
        {
          const auto& minDistToEachObst = myModel->getMinDistToEachObstacle();
          auto idx = isParam1 ? 0 : 1;
          mDummy[idx].clear();
          for (size_t i(0); i < minDistToEachObst.size(); ++i)
          {
            mDummy[idx].push_back(minDistToEachObst[i][obstacleIndex]);
          }
          parameterList  = &mDummy[idx];
          parameterOrder = &myModel->getMinDistToEachObstacleOrder()[obstacleIndex];
        }
      }
      mpParetoWidget->setParetoParameter(isParam1, parameterName, parameterList, parameterOrder, filteredPaths);
    }

    /** resets both parameters to '- not selected -' (used when the amount of filtered paths changed)
    */
    void SelectionController::resetParetoFront()
    {
      if (mpParetoWidget)
        mpParetoWidget->resetChoice();
    }

    /**
    */
    void SelectionController::setDefaultSelectionView()
    {
      if (mActiveKey.empty())
        return;
      auto pColl = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey);
      const auto N = pColl->numberOfPaths();
      for (size_t i(0); i < N; ++i)
      {
        auto pPath = pColl->getMukPath(i);
        pPath->setColors(pPath->getDefaultColor());
        pPath->setLineWidth(1.0);
        pPath->setVisibility(true);
        pPath->setOpacity(1);
        pPath->setTopology(VisMukPath::EnTopology::Lines);
      }
    }

    /**
    */
    void SelectionController::markPathAsSelected(size_t pathIdx, size_t oldIdx)
    {
      auto pColl = mpModels->pVisModel->getVisScene()->getPathCollection(mActiveKey);
      if(oldIdx != -1)
        pColl->getMukPath(oldIdx)->setTopology(VisMukPath::EnTopology::Lines);
      if(pathIdx != -1)
        pColl->getMukPath(pathIdx)->setTopology(VisMukPath::EnTopology::Tube);
    }

    /** \brief sets the visibility of all paths of the given collection to true/false
    */
    void SelectionController::setCollectionVisibility(const std::string& key, bool visible)
    {
      auto pColl = mpModels->pVisModel->getVisScene()->getPathCollection(key);
      const auto N = pColl->numberOfPaths();
      for (size_t i(0); i<N; ++i)
      {
        pColl->getMukPath(i)->setVisibility(visible);
      }
    }

  }
}