#include "private/muk.pch"

#include "ApplicationLogic.h"
#include "CtVisLogic.h"
#include "SelectionLogic.h"
#include "VisualizationLogic.h"
#include "PlanningLogic.h"

#include "MukCommon/MukException.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/geometry.h"

#include "MukImaging/muk_imaging_tools.h"

#include "MukObstacle.h"

#include "MukEvaluation/statistics.h"

#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisObstacle.h"
#include "MukVisualization/muk_colors.h"
#include "MukVisualization/PolyDataHandler.h"

#include "MukQt/muk_qt_tools.h"
#include "MukQt/TabSelection.h"
#include "MukQt/SceneWidget.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "gris_math.h"

#include <QtWidgets/QTabWidget>

#include <vtkSmartPointer.h>

#include <vtkImageReslice.h>
#include <vtkImplicitPlaneWidget2.h>
#include <vtkCubeSource.h>
#include <vtkImplicitPlaneRepresentation.h>
#include <vtkImageResliceMapper.h>
#include <vtkImageProperty.h>
#include <vtkImageSlice.h>
#include <vtkProperty.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkPlane.h>
#include <vtkExtractVOI.h>


/**
PathEvaluator Class Def
*/
namespace gris
{
  namespace muk
  {
    class PathEvaluator
    {
    public:
      PathEvaluator(const std::vector<double> weightings, MukScene& mpScene);

    public:
      // brings together all results of the different categories and weights them against each other
      double EvaluatePath(const double* diffValues) const;
      // calculates the distances of all paths to all different obstacles with differing weightings between those obstacles
      std::vector<double> WeightedDistances(VisPathCollection& paths, const double rad) const;
      // adds all curvatures of a path 
      double PathStraightness(const std::vector<MukState> path, const std::vector<MukState> goalStates) const;
      // determines the difference between the ideal angle of arrival and the actual angle of arrival
      double GoalAngleDifference(const MukState& pathEndState, const std::vector<MukState> goalStates) const;
      // analyses the air-to-bone-ratio in the surroundings along the path and recognizes airholes along the path.
      // also calculates the bone density along the path but doesn't return it right now. Further testing is needed to see what is important
      double TextureSpecifics(const std::vector<MukState> path, const itk::SmartPointer<itk::Image<unsigned short, 3U>> ctImage, const double rad, const std::vector<double> holeVar) const;
      std::vector<double> getCategoryWeightings() { return this->categoryWeightings; };
      std::vector<double> getobstacleWeightings() { return this->obstacleWeightings; };

    private:
      MukScene* mpScene;
      // saves the weightings for the different categories
      std::vector<double> categoryWeightings;
      // saves the weightings for the different obstacles
      std::vector<double> obstacleWeightings;
      const double maxDist = std::numeric_limits<double>::infinity();
    };
  }
}

/**
PaintImageCallback Class Def

Die Klasse haben wir ja zusammen nur aus dem Code eines deiner Mitarbeiter kopiert, also hab ich da ja eigentlich nix selber geschrieben.
Deswegen nehm ich mir nicht heraus das zu kommentieren und so zu tun als hätte ich zu der Klasse irgendwas beigesteuert.
Wenn du willst kann ich die aber noch kommentieren.
*/
namespace gris
{
  namespace muk
  {
    class PaintImageCallback : public vtkCommand
    {
    public:
      static PaintImageCallback* New();
      vtkTypeMacro(PaintImageCallback, vtkCommand);

    public:
      PaintImageCallback()
      {

        mpReslice = make_vtk<vtkImageReslice>();
        mpReslice->SetOutputDimensionality(3);
        mpReslice->SetInterpolationModeToLinear();

        mpPlane = make_vtk<vtkPlane>();

        mpResliceMapper = make_vtk<vtkImageResliceMapper>();
        mpResliceMapper->SliceFacesCameraOff();
        mpResliceMapper->SliceAtFocalPointOff();
        mpResliceMapper->BorderOn();
        mpResliceMapper->SetInputConnection(mpReslice->GetOutputPort());

        mpIP = make_vtk<vtkImageProperty>();
        mpIP->SetColorWindow(2000);
        mpIP->SetColorLevel(1000);
        mpIP->SetAmbient(0.0);
        mpIP->SetDiffuse(1.0);
        mpIP->SetOpacity(1.0);
        mpIP->SetInterpolationTypeToLinear();

        mpSlice = make_vtk<vtkImageSlice>();
        mpSlice->SetProperty(mpIP);
        mpSlice->SetMapper(mpResliceMapper);
      }

      void setRenderer(vtkRenderer* pObj) {
        mpRenderer = pObj;
        mpRenderer->AddViewProp(mpSlice);
      }
      void setImage(vtkImageData* pObj) {
        mpVolumeImage = pObj;
        mpReslice->SetInputData(mpVolumeImage);
      }
      void SetRepresentation(vtkImplicitPlaneRepresentation* pRep) {
        mpRep = pRep;
        // einzig das hier hab ich geändert
        // Disables all visual parts of the widget
        mpRep->SetVisibility(false);
        //mpRep->GetPlaneProperty()->SetOpacity(0.1);
        //mpRep->GetPlaneProperty()->SetDiffuse(0.0);
      }
      // ManualUpdate hast du gemacht (wobei das auch nur alles von execute verschoben ist)
      void ManualUpdate()
      {
        mpRep->GetPlane(mpPlane);
        mpResliceMapper->SetSlicePlane(mpPlane);
        mpResliceMapper->Modified();
        mpRep->SetOutsideBounds(false);
        mpSlice->Update();
        mpRenderer->Render();
      }
      // und das hier
      // sets the vibility of the CT-Slice
      void SetVisibility(bool b)
      {
        mpSlice->SetVisibility(b);
      }

    public:
      virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData)
      {
        ManualUpdate();
      }

    private:
      vtkSmartPointer<vtkImageReslice> mpReslice;
      vtkSmartPointer<vtkImageData> mpVolumeImage;
      vtkSmartPointer<vtkTransform> mpTransform;
      vtkSmartPointer<vtkPlane> mpPlane;
      vtkRenderer* mpRenderer;
      vtkSmartPointer<vtkImageResliceMapper> mpResliceMapper;
      vtkSmartPointer<vtkImageProperty> mpIP;
      vtkSmartPointer<vtkImageSlice> mpSlice;
      vtkImplicitPlaneRepresentation* mpRep;
    };
    vtkStandardNewMacro(PaintImageCallback);
  }
}

namespace gris
{
  namespace muk
  {
    /**
    */
    SelectionLogic::SelectionLogic(std::shared_ptr<MukScene> pScene)
      : BaseLogic()
      , mpTabSelection(nullptr)
      , mpImpPlaneWidget(make_vtk<vtkImplicitPlaneWidget2>())
      , mpPlaneRep ( make_vtk<vtkImplicitPlaneRepresentation>())
      , mpCallback ( make_vtk<PaintImageCallback>())
      , mpToVtk    ( make_itk<itk::ImageToVTKImageFilter<MukImage>>())
    {

    }

    /** \brief to be moved to a controller
    */
    void SelectionLogic::setupConnections(MuknoPlannerMainWindow* pWindow)
    {
      if (nullptr == pWindow)
        throw MUK_EXCEPTION_SIMPLE("empty pointer was passed");

      mpPlanningLogic = mpLogics->pPlanningLogic.get();
      mpTabSelection = pWindow->mpTabSelection;
      mpPropertyWidget = pWindow->mpPropertyWidget;
      mpScene = mpLogics->pAppLogic->getScene().get();
      mpVisScene = mpLogics->pVisLogic->getVisScene();
      mpTabContainer = pWindow->mpTabContainer;
      // weightings for: distance to riskstructures, path Straightness, goal angle, density along the path and the 12 obstacles
      weightings = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
      // threshold (Values under that th are considered air) | ratio of pixel under the threshold and 
      // over the threshold to consider the next state a hole | ratio of pixel under the th and over,
      // to rate the hole as over, thus creating a hysteresis |
      // length of the hole in mm to be considered a threat to the drill
      holeVariables = { 500, 0.9, 0.8, 3 }; 

      connect(mpTabSelection, &TabSelection::resetSelectionClicked, this, &SelectionLogic::resetSelection);
      connect(mpTabSelection, &TabSelection::colorPathsClicked, this, &SelectionLogic::colorPaths);
      connect(mpTabSelection, &TabSelection::clickedEvaluatePath, this, &SelectionLogic::evaluatePaths);
      connect(mpTabSelection, &TabSelection::categoryWeightingChanged, this, &SelectionLogic::categoryWeightingChanged);
      connect(mpTabSelection, &TabSelection::obstacleWeightingChanged, this, &SelectionLogic::obstacleWeightingChanged);
      connect(mpTabSelection, &TabSelection::singlePathSelectionChanged, this, &SelectionLogic::highlightSinglePath);
      // Reloading the ParameterNets when switching tabs and when changing properties in the property window
      connect(mpTabContainer, &QTabWidget::currentChanged, this, &SelectionLogic::propertyChanged);
      connect(mpPropertyWidget, &PropertyWidget::propertyChanged, this, &SelectionLogic::propertyChanged);
      connect(mpTabSelection, &TabSelection::ctOverlayClicked, this, &SelectionLogic::toggleCTOverlay);
      connect(mpTabSelection, &TabSelection::ctOverlayScrolled, this, &SelectionLogic::scrollCTOverlay);
      connect(mpTabSelection, &TabSelection::displayedStateChanged, this, &SelectionLogic::ctStateChanged);
      // Displaying the Slices of the new path when the active path is changed
      connect(mpPlanningLogic, &PlanningLogic::activePathIndexChanged, this, &SelectionLogic::displayCTSlice);

      connect(mpTabSelection, &TabSelection::clickedShowOnly, this, SELECT<void>::OVERLOAD_OF(&SelectionLogic::showSinglePath));
    }

    /**
    */
    void SelectionLogic::resetSelection()
    {
      resetPaths();
      mpLogics->pVisLogic->render();
    }

    /**
    */
    void SelectionLogic::resetPaths()
    {
      const auto& key = mpLogics->pPlanningLogic->getActivePathCollection();
      auto pColl = mpVisScene->getPathCollection(key);
      const auto N = pColl->numberOfPaths();
      for (size_t i(0); i < N; ++i)
      {
        auto pPath = pColl->getMukPath(i);
        pPath->setColors(pPath->getDefaultColor());
        pPath->setLineWidth(1.0);
        pPath->setVisibility(true);
        pPath->setOpacity(1);
      }
      mpPlanningLogic->setActivePathIdx(-1);
    }

    /**
    The method that is called when clicking the corresponding button
    Calls the calculations for all 4 categories with the set weightings and calls evaluatePath to bring them together and rate a path. 
    */
    void SelectionLogic::evaluatePaths()
    {
      const auto& key = mpPlanningLogic->getActivePathCollection();
      double rad = mpScene->getPathCollection(key).getProblemDefinition()->getRadius();
      const auto paths = mpVisScene->getPathCollection(key);
      auto numberOfPaths = paths->numberOfPaths();
      if (numberOfPaths == 0)
      {
        LOG_LINE << "No Paths in Collection";
        return;
      }
      PathEvaluator pEval(weightings, *mpScene);
      if (pEval.getCategoryWeightings()[0] == 0 && pEval.getCategoryWeightings()[1] == 0 &&
        pEval.getCategoryWeightings()[2] == 0 && pEval.getCategoryWeightings()[3] == 0)
      {
        std::string err = "All Category-Weightings are zero, so the path can't be judged";
        std::string info = "Category-Weightings: ObstacleDistance " + std::to_string((int)pEval.getCategoryWeightings()[0]) + " PathStraightness "
          + std::to_string((int)pEval.getCategoryWeightings()[1]) + " GoalAngle " + std::to_string((int)pEval.getCategoryWeightings()[2]) + " BoneToAirRatio "
          + std::to_string((int)pEval.getCategoryWeightings()[3]);
        throw MUK_EXCEPTION(err.c_str(), info.c_str());
      }
      auto ctImage = mpLogics->pCtVisLogic->getImage();
      auto pCT = mpLogics->pCtVisLogic->getCtData();
      // Only aborts the calculations when TextureSpecifics weighted more then 0
      if (pCT == nullptr && pEval.getCategoryWeightings()[3] != 0)
      {
        LOG_LINE << "no CT-Data loaded yet!";
        return;
      }
      // A vector with double the capacity is needed because pEval.WeightedDistances returns both the distances with weighted Obstacles and the true distances
      // Fill the Vectors with the worst possible values
      std::vector<double> weightedDistances(numberOfPaths * 2);
      weightedDistances.assign(numberOfPaths * 2, 0);
      std::vector<double> pathStraightness(numberOfPaths);
      pathStraightness.assign(numberOfPaths, maxDist);
      std::vector<double> goalAngles(numberOfPaths);
      goalAngles.assign(numberOfPaths, maxDist);
      std::vector<double> textureDensity(numberOfPaths);
      textureDensity.assign(numberOfPaths, maxDist);
      // only refill them with proper values if the corresponding weighting is bigger than 0 to save on calculations
      if(pEval.getCategoryWeightings()[0] > 0)
        weightedDistances = pEval.WeightedDistances(*paths, rad);
      for (size_t i(0); i < numberOfPaths; ++i)
      {
        if(pEval.getCategoryWeightings()[1] > 0)
          pathStraightness[i] = pEval.PathStraightness(paths->getMukPath(i)->asMukPath().getPath(), mpScene->getPathCollection(key).getProblemDefinition()->getGoalStates());
        if (pEval.getCategoryWeightings()[2] > 0)
          goalAngles[i] = pEval.GoalAngleDifference(paths->getMukPath(i)->asMukPath().getPath().back(), mpScene->getPathCollection(key).getProblemDefinition()->getGoalStates());
        if (pEval.getCategoryWeightings()[3] > 0)
          textureDensity[i] = pEval.TextureSpecifics(paths->getMukPath(i)->asMukPath().getPath(), pCT, rad, holeVariables);
        // If a airhole was detected along the path the worst possible values will be assigned to the path to make sure it wont be chosen 
        if (textureDensity[i] == -1)
        {
          LOG_LINE << "Path " << i + 1 << " can't be used because there is an airhole in the way";
          weightedDistances[i] = 0;
          pathStraightness[i] = maxDist;
          goalAngles[i] = maxDist;
          textureDensity[i] = maxDist;
          // hides the unusable paths
          paths->getMukPath(i)->setVisibility(false);
        }
      }
      // vectors for the absolute Values of the paths for future readout
      std::vector<double> absoluteWD;
      // using the unweighted distances to the obstacles to see if a lesser weighted obstacle is closer to the robot then the more weighted obstacle
      absoluteWD.assign(weightedDistances.begin() + numberOfPaths, weightedDistances.end());
      weightedDistances.erase(weightedDistances.begin() + numberOfPaths, weightedDistances.end());
      std::vector<double> absolutePS = pathStraightness;
      std::vector<double> absoluteGA = goalAngles;
      std::vector<double> absoluteTD = textureDensity;
      // best Distance Index, best Straightness Index, best Angle Index,  best Texture Index
      size_t bDI = std::distance(weightedDistances.begin(), std::max_element(weightedDistances.begin(), weightedDistances.end()));
      size_t bSI = std::distance(pathStraightness.begin(), std::min_element(pathStraightness.begin(), pathStraightness.end()));
      size_t bAI = std::distance(goalAngles.begin(), std::min_element(goalAngles.begin(), goalAngles.end()));
      size_t bTI = std::distance(textureDensity.begin(), std::min_element(textureDensity.begin(), textureDensity.end()));
      // If all paths in the PathCollection can't be used because of airholes the calculations will be aborted
      // that can be detected when all paths have the worst value in a category that has a weighting of bigger than 0
      if (weightedDistances[bDI] == 0 && pEval.getCategoryWeightings()[0] > 0 ||
        pathStraightness[bSI] == maxDist && pEval.getCategoryWeightings()[1] > 0 ||
        goalAngles[bAI] == maxDist && pEval.getCategoryWeightings()[2] > 0||
        textureDensity[bTI] == maxDist && pEval.getCategoryWeightings()[3] > 0)
      {
        LOG_LINE << "None of the available paths is viable due to airholes";
        return;
      }
      // create relativ values for all indices other then the best in each category. Every index relativ to the best
      for (size_t i(0); i < numberOfPaths; ++i)
      {
        if (i != bDI)
          weightedDistances[i] = weightedDistances[i] / weightedDistances[bDI];
        if (i != bSI)
          pathStraightness[i] = pathStraightness[bSI] / pathStraightness[i];
        if (i != bAI)
          goalAngles[i] = goalAngles[bAI] / goalAngles[i];
        if (i != bTI)
          textureDensity[i] = textureDensity[bTI] / textureDensity[i];
      }
      // the best Values are set to 1
      weightedDistances[bDI] = 1;
      pathStraightness[bSI] = 1;
      goalAngles[bAI] = 1;
      textureDensity[bTI] = 1;
      std::vector<double> pathRatings(numberOfPaths);
      // rating the path according to the weightings with the 4 relativ values for each category
      // the relativ values are needed to make all categories comparable
      // fill the values with 0 if there is no weighting
      for (size_t i(0); i < numberOfPaths; ++i)
      {
        double pValues[4] = { pEval.getCategoryWeightings()[0] > 0 ? weightedDistances[i] : 0, pEval.getCategoryWeightings()[1] > 0 ? pathStraightness[i] : 1
          , pEval.getCategoryWeightings()[2] > 0 ? goalAngles[i] : 0, pEval.getCategoryWeightings()[3] > 0 ? textureDensity[i] : 0 };
        pathRatings[i] = pEval.EvaluatePath(pValues);
      }
      size_t bestPathIndex = std::distance(pathRatings.begin(), std::max_element(pathRatings.begin(), pathRatings.end()));
      // hab ich zum bugfixen benutzt, kann aber eigentlich auch ausgegeben werden um zu sehen was das weighting gebracht hat
      LOG << "Relativ: " << weightedDistances[bestPathIndex] << " | " << pathStraightness[bestPathIndex] << "\n";
      LOG << goalAngles[bestPathIndex] << " | " << textureDensity[bestPathIndex] << "\n";
      LOG << "PathScore: " << pathRatings[bestPathIndex] << "\n";

      LOG << "Best weighted Path is Path Nr. " << bestPathIndex + 1 << "\n";
      // see if an lesser weighted obstacle is closer to the path then the higher weighted obstacles
      std::vector<double> normalPathDist = computeDistances(*mpScene->getCollisionDetector(), paths->getMukPath(bestPathIndex)->asMukPath().getPath(), rad);
      size_t normalMaxDist = std::distance(normalPathDist.begin(), std::min_element(normalPathDist.begin(), normalPathDist.end()));
      if (normalPathDist[normalMaxDist] < absoluteWD[bestPathIndex])
        LOG << "Min Dist to Riskstructures: " << absoluteWD[bestPathIndex] << " but a less weighted Obstacle is closer with: " << normalPathDist[normalMaxDist] << "\n";
      else
        LOG << "Min Dist to Riskstructures: " << absoluteWD[bestPathIndex] << "\n";
      LOG << "Min summ of all Curvatures: " << absolutePS[bestPathIndex] << "\n";
      LOG << "Angle between the path and the wanted entry-direction: " << absoluteGA[bestPathIndex] << "\n";
      LOG_LINE << "Average air to bone ratio along the path: " << absoluteTD[bestPathIndex];
      highlightSinglePath(bestPathIndex);
    }

    /**
    */
    void SelectionLogic::highlightSinglePath(size_t idx)
    {
      const auto& key = mpPlanningLogic->getActivePathCollection();
      auto pColl = mpVisScene->getPathCollection(key);
      const auto N = pColl->numberOfPaths();
      for (size_t i(0); i < N; ++i)
      {
        auto pPath = pColl->getMukPath(i);
        if (i == idx)
        {
          pPath->setOpacity(1.0);
          pPath->setLineWidth(5.0);
          // make sure, it is visible
          pPath->setVisibility(true);
        }
        else
        {
          pPath->setOpacity(0.33);
          pPath->setLineWidth(1.0);
        }
      }
      mpTabSelection->setSinglePathSelection(idx);
      mpPlanningLogic->setActivePathIdx(idx);
      mpLogics->pVisLogic->render();
    }

    /**
    Changes the categoryWeightings when signaled by the spinboxes in TabSelection or the drag and drop of the ParameterNet
    */
    void SelectionLogic::categoryWeightingChanged(double d, int i)
    {
      weightings[0 + i] = d;
    }

    /**
    Changes the obstacleWeightings when signaled by the spinboxes in TabSelection or the drag and drop of the ParameterNet
    */
    void SelectionLogic::obstacleWeightingChanged(double d, int i)
    {
      weightings[4 + i] = d;
    }

    /**
    When signaled by changing the properties in the propertyWidget or by switching the tabs of the TabContainer
    this Method gathers the colors of the obstacles from the propertyWidget and updates the Obstacles used in TabSelection
    */
    void SelectionLogic::propertyChanged()
    {
      std::vector<QColor> colors;
      auto obstacleList = mpVisScene->getObstacleKeys();
      for (size_t i(0); i < obstacleList.size(); ++i)
      {
        colors.push_back(QColor(mpVisScene->getObstacle(obstacleList[i])->getDefaultColor().x() * 255,
          mpVisScene->getObstacle(obstacleList[i])->getDefaultColor().y() * 255,
          mpVisScene->getObstacle(obstacleList[i])->getDefaultColor().z() * 255));
      }
      mpTabSelection->updateObstacles(colors, *mpScene);
    }

    /**
    */
    void SelectionLogic::showSinglePath()
    {
      size_t idx = mpTabSelection->getSinglePathSelection();
      showSinglePath(idx);
    }

    /**
    */
    void SelectionLogic::showSinglePath(size_t idx)
    {
      bool ctWasActive = ctOverlayActive;
      // the CT-Overlay gets deactivated by resetting paths (because no path is selected) but resetting paths is not the goal of this method
      // so the Overlay has to be reactivated if it was activ beforehand
      resetPaths();
      const auto& key = mpPlanningLogic->getActivePathCollection();
      auto pColl = mpVisScene->getPathCollection(key);
      const auto N = pColl->numberOfPaths();
      for (size_t i(0); i < N; ++i)
      {
        if (i != idx)
          pColl->getMukPath(i)->setVisibility(false);
        else
          pColl->getMukPath(i)->setLineWidth(5.0);
      }
      mpPlanningLogic->setActivePathIdx(idx);
      if(ctWasActive)
        toggleCTOverlay();
      mpLogics->pVisLogic->render();
    }

    /** \brief color codes the paths according to the distance to risk structures

    todo: should perform color coding according to different color maps
    */
    void SelectionLogic::colorPaths()
    {
      const auto& key = mpPlanningLogic->getActivePathCollection();
      auto pVisColl = mpVisScene->getPathCollection(key);
      const auto N = pVisColl->numberOfPaths();
      auto& coll = mpScene->getPathCollection(key);
      double r = coll.getProblemDefinition()->getRadius();
      if (N == 0)
        return;

      const auto& detector = *mpScene->getCollisionDetector();
      if (detector.isOutOfDate())
        detector.rebuild();

      std::vector< std::pair<size_t, double>> distIdxPair(N);
      for (size_t i(0); i < N; ++i)
      {
        auto vecDists = computeDistances(detector, pVisColl->getMukPath(i)->asMukPath().getPath(), r);
        double maxDist = std::numeric_limits<double>::max();
        if (!vecDists.empty())
        {
          maxDist = *min_element(vecDists.begin(), vecDists.end());
        }
        distIdxPair[i].first = i;
        distIdxPair[i].second = maxDist;
      }
      std::sort(distIdxPair.begin(), distIdxPair.end(), [&](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
      LOG_LINE << "Minimal largest distance: idx = " << distIdxPair.back().first << ", , dist = " << distIdxPair.back().second;

      double color[3];
      const double N_2 = N*0.5;
      for (size_t i(0); i < static_cast<size_t>(N_2); ++i)
      {
        color[0] = Colors::Red[0];
        color[1] = Colors::Red[1] + i*1.0 / N_2;
        color[2] = Colors::Red[2];
        pVisColl->getMukPath(distIdxPair[i].first)->setColors(color);
        LOG_LINE << "index " << distIdxPair[i].first << ", dist = " << distIdxPair[i].second;
      }
      for (size_t i(static_cast<size_t>(N_2)); i < N; ++i)
      {
        color[0] = Colors::Yellow[0] - (i - N_2) * 1 / N_2;
        color[1] = Colors::Green[1];
        color[2] = Colors::Green[2];
        pVisColl->getMukPath(distIdxPair[i].first)->setColors(color);
        LOG_LINE << "index " << distIdxPair[i].first << ", dist = " << distIdxPair[i].second;
      }
      mpLogics->pVisLogic->render();
    }

    /**
    When signaled by TabSelection the CtOverlay will be activated or deactivated
    */
    void SelectionLogic::toggleCTOverlay()
    {
      // deactivation
      if (ctOverlayActive)
      {
        mpTabSelection->toggleCTOverlay(ctOverlayActive);
        displayCTSlice();
        return;
      }
      // activation
      auto pCT = mpLogics->pCtVisLogic->getCtData();
      if (pCT == nullptr)
      {
        LOG_LINE << "no CT-Data loaded yet!";
        return;
      }
      if (mpPlanningLogic->getActivePathIdx() == mpPlanningLogic->Invalid_Index)
      {
        LOG_LINE << "no path chosen yet";
        return;
      }
      mpTabSelection->toggleCTOverlay(ctOverlayActive);
      displayCTSlice();
    }

    /**
    When the CTOverlay is active and while scrolling on the SelectionTab the positionInPath is changed accordingly
    */
    void SelectionLogic::scrollCTOverlay(bool forward)
    {
      if (ctOverlayActive)
      {
        const auto& key = mpPlanningLogic->getActivePathCollection();
        auto pathCollection = mpVisScene->getPathCollection(key);
        auto activePath = pathCollection->getMukPath(mpPlanningLogic->getActivePathIdx())->asMukPath().getPath();
        // won't increase the positionInPath when the position is at the end of the path
        if (forward && activePath.size() > positionInPath + 1)
        {
          positionInPath++;
        }
        // won't decrease the positionInPath when the position is at the start of the path
        else if (!forward &&  positionInPath > 0)
        {
          positionInPath--;
        }
        mpTabSelection->updateDisplayState(positionInPath, activePath.size());
        displayCTSlice();
      }
    }

    /**
    Called when the CT-State is changed via the spinbox in TabSelection and adjusts the positionInPath accordingly
    calls updateDisplayState to update the spinbox.
    */
    void SelectionLogic::ctStateChanged(size_t i)
    {
      if (ctOverlayActive)
      {
        auto currentPath = mpVisScene->getPathCollection(mpPlanningLogic->getActivePathCollection())->getMukPath(mpPlanningLogic->getActivePathIdx())->asMukPath().getPath();
        positionInPath = i;
        mpTabSelection->updateDisplayState(positionInPath, currentPath.size());
        displayCTSlice();
      }
      else
      {
        LOG_LINE << "CT-Overlay not active";
        mpTabSelection->updateDisplayState(positionInPath);
      }
    }

    /**
    Deactivating the Display when the CT-Overlay is inactive or when the paths have been resetted
    Activating the Display of the State with the Index positionInPath or changing the State when it's already active
    */
    void SelectionLogic::displayCTSlice()
    {
      //remove old Slice
      if (mpPlanningLogic->getActivePathIdx() == mpPlanningLogic->Invalid_Index && ctOverlayActive)
      {
        mpImpPlaneWidget->Off();
        mpCallback->SetVisibility(false);
        mpLogics->pVisLogic->render();
        toggleCTOverlay();
        return;
      }
      if (!ctOverlayActive)
      {
        mpImpPlaneWidget->Off();
        mpCallback->SetVisibility(false);
        mpLogics->pVisLogic->render();
        return;
      }
      //display new Slice
      auto currentPath = mpVisScene->getPathCollection(mpPlanningLogic->getActivePathCollection())->getMukPath(mpPlanningLogic->getActivePathIdx())->asMukPath().getPath();
      if (currentPath.size() <= positionInPath)
      {
        // at switching the active path and when the positionInPath is bigger then the biggest index in the new path, positionInPath is shortend accordingly
        positionInPath = currentPath.size() - 1;
        mpTabSelection->updateDisplayState(positionInPath, currentPath.size());
        LOG_LINE << "Shortened the position in path according to the new path";
      }
      LOG_LINE << "Displaying CT-Slice of State " << positionInPath + 1<< " of Path " << mpPlanningLogic->getActivePathIdx();
      auto ctImage = mpLogics->pCtVisLogic->getImage();
      auto pCT = mpLogics->pCtVisLogic->getCtData();

      // Auch hier werd ich nur Sachen kommentieren die ich anders gemacht habe als das war wir aus dem anderen Code kopiert haben
      mpImpPlaneWidget->SetInteractor(mpLogics->pVisLogic->getVisWindow()->getVtkWindow()->GetRenderWindow()->GetInteractor());
      const auto& key = mpPlanningLogic->getActivePathCollection();
      double rad = mpScene->getPathCollection(key).getProblemDefinition()->getRadius();
      double viewCubeSize = 10;
      auto pBoundingBox = make_vtk<vtkCubeSource>();
      {
        // A cube with edge length of 10 * the radius of the drilling robot
        pBoundingBox->SetCenter(currentPath[positionInPath].coords.x(), currentPath[positionInPath].coords.y(), currentPath[positionInPath].coords.z());
        pBoundingBox->SetXLength(viewCubeSize * rad);
        pBoundingBox->SetYLength(viewCubeSize * rad);
        pBoundingBox->SetZLength(viewCubeSize * rad);
        pBoundingBox->Update();
      }
      mpPlaneRep->PlaceWidget(pBoundingBox->GetOutput()->GetBounds());
      mpPlaneRep->SetPlaceFactor(1.0);
      // The Plane will be fixed at the current State in the current path with the Normal of the State's tangent
      mpPlaneRep->SetNormal(currentPath[positionInPath].tangent.data());
      mpPlaneRep->SetOrigin(currentPath[positionInPath].coords.data());
      mpImpPlaneWidget->SetRepresentation(mpPlaneRep);
      // Disables any interaction with the Widget
      mpImpPlaneWidget->SetProcessEvents(0);

      mpToVtk->SetInput(pCT);
      mpToVtk->Update();
      auto renderer = mpLogics->pVisLogic->getVisWindow()->getVtkWindow()->getRenderer();

      mpCallback->setRenderer(renderer);
      // class to focus on a small part of a larger data-cube, effectively cropping it
      vtkSmartPointer<vtkExtractVOI> cropper;
      cropper = make_vtk<vtkExtractVOI>();
      cropper->SetInputData(mpToVtk->GetOutput());
      double centerPoint[3] = { currentPath[positionInPath].coords.x(), currentPath[positionInPath].coords.y(), currentPath[positionInPath].coords.z() };
      // cropping the image to the size of the bounds
      double pointLow[3] = { centerPoint[0] - viewCubeSize/2 * rad, centerPoint[1] - viewCubeSize/2 * rad, centerPoint[2] - viewCubeSize/2 * rad };
      double pointHigh[3] = { centerPoint[0] + viewCubeSize/2 * rad, centerPoint[1] + viewCubeSize/2 * rad, centerPoint[2] + viewCubeSize/2 * rad };
      itk::Point<double> pLow = pointLow;
      itk::Point<double> pHigh = pointHigh;
      itk::Index<3U> iLow;
      itk::Index<3U> iHigh;
      // transform the bounds to indices and crop the data-cube
      pCT->TransformPhysicalPointToIndex(pLow, iLow);
      pCT->TransformPhysicalPointToIndex(pHigh, iHigh);
      cropper->SetVOI(iLow.GetIndex()[0], iHigh.GetIndex()[0], iLow.GetIndex()[1], iHigh.GetIndex()[1], iLow.GetIndex()[2], iHigh.GetIndex()[2]);
      cropper->Update();

      mpCallback->setImage(cropper->GetOutput());
      mpImpPlaneWidget->On();
      mpCallback->SetRepresentation(mpPlaneRep);
      mpImpPlaneWidget->AddObserver("InteractionEvent", mpCallback);
      mpCallback->ManualUpdate();
      // make sure that the Slice is visible
      mpCallback->SetVisibility(true);
      mpLogics->pVisLogic->render();
    }
  }
}
//Implementation of the PathEvaluator class
namespace gris
{
  namespace muk
  {
    /**
    Assigning the weightings to the categoryWeightings and the obstacleWeightings
    */
    PathEvaluator::PathEvaluator(const std::vector<double> weightings, MukScene& Scene)
    {
      mpScene = &Scene;
      // Distance, Straightness, GoalAngle, Texture
      categoryWeightings.assign(weightings.begin(), weightings.begin() + 4);
      obstacleWeightings.assign(weightings.begin() + 4, weightings.end());
    }

    /**
    normal weightingfunction for the 4 categoryValues with the 4 categoryWeightings
    */
    double PathEvaluator::EvaluatePath(const double* categoryValues) const
    {
      double pathScore(0);
      double weightingSumm(0);
      for (size_t i(0); i < 4; ++i)
      {
        pathScore += categoryValues[i] * categoryWeightings[i];
        weightingSumm += categoryWeightings[i];
      }
      return pathScore/weightingSumm;
    }

    /**
    Calculates the distances to all Obstacles while weighting them against each other
    */
    std::vector<double> PathEvaluator::WeightedDistances(VisPathCollection& paths, const double rad) const
    {
      size_t pathN = paths.numberOfPaths();
      // The vector has double the capacity of the number of paths because both the distances with weighted Obstacles and the true distances are needed
      std::vector<double> result(pathN*2);
      ICollisionDetector& detector = *mpScene->getCollisionDetector();
      const std::vector<std::string> obstKeys = detector.getKeys();
      // vector of Obstacles, vector of Paths, vector of Distance to each State per Obstacle
      std::vector<std::vector<std::vector<double>>> unweightedPathDist;
      // save active Obstacles in order to restore them later on and to save on processing
      std::vector<bool> activeObstacles; 
      // save the Obstacle weightings to maintain order later on
      std::vector<double> tempWeightings; 
      for (int i = 0; i < (int)obstKeys.size(); ++i)
      {
        activeObstacles.push_back(detector.isActive(obstKeys[i]));
      }
      bool allWeightingsZero = true;
      std::string info = "Obstacle-Weightings: ";
      for (size_t j(0); j < obstacleWeightings.size(); ++j)
      {
        if (activeObstacles[j])
        {
          if (obstacleWeightings[j] > 0)
            allWeightingsZero = false;
          info += " " + obstKeys[j] + ": " + std::to_string((int)obstacleWeightings[j]);
        }
      }
      if (allWeightingsZero)
      {
        std::string err = "All Obstacle-Weightings are zero, so the path can't be judged";
        throw MUK_EXCEPTION(err.c_str(), info.c_str());
      }
      for (int i(0); i < (int)obstKeys.size(); ++i) // For every Obstacle i,
      {
        if (activeObstacles[i] && obstacleWeightings[i] > 0) // that was active in the beginning and that has a bigger Weighting than 0,
        {
          tempWeightings.push_back(obstacleWeightings[i]);
          
          for (int j(0); j < (int)obstKeys.size(); ++j) // deactivate each other Obstacle
          {
            if (obstKeys[j] == obstKeys[i])
              detector.setActive(obstKeys[j], true);
            else
              detector.setActive(obstKeys[j], false);
          }
          detector.rebuild(); // and Rebuild the Tree
          std::vector<std::vector<double>> tempPaths;
          // filler to enable parallel processing
          std::vector<double> filler;
          tempPaths.assign(pathN, filler);
#pragma omp parallel for
          // Search nearest Neighbors along all paths for that one Obstacle
          for (int j(0); j < (int)pathN; ++j) // Path j
          {
            tempPaths[j] = computeDistances(*mpScene->getCollisionDetector(), paths.getMukPath(j)->asMukPath().getPath(), rad);
          }
          unweightedPathDist.push_back(tempPaths);
        }
      }
      // restore the prior active Obstacles
      for (int i(0); i < (int)obstKeys.size(); ++i) 
      {
        detector.setActive(obstKeys[i], activeObstacles[i]);
      }
      // rebuild the tree
      detector.rebuild();
      result.assign(pathN*2, 0);
#pragma omp parallel for
      for (int i(0); i < (int)pathN; ++i) // every Path i
      {
        std::vector<double> weightedMinDistPerState; // weighted min dist Values for every State of Path i
        std::vector<double> unweightedMinDistPerState; // unweighted min dist Values for every State of Path i
        weightedMinDistPerState.assign(paths.getMukPath(i)->asMukPath().getPath().size(), maxDist);
        unweightedMinDistPerState.assign(paths.getMukPath(i)->asMukPath().getPath().size(), maxDist);
#pragma omp parallel for
        for (int j(0); j < (int)paths.getMukPath(i)->asMukPath().getPath().size(); ++j) // every State j
        {
          std::vector<double> tempDistPerObst; // All per-Obstacle-weighted Values of State j
          tempDistPerObst.assign(unweightedPathDist.size(), maxDist);
#pragma omp parallel for
          for (int l(0); l < (int)unweightedPathDist.size(); ++l) // every Obstacle l
          {
            // Substracting the minimal safety distance of 0.5 from every distance to increase the effect of the weighting
            // Distances of weighted obstacles are decreased so they are more often the minimal element
            tempDistPerObst[l] = (std::max(0.0,unweightedPathDist[l][i][j]-0.5)) / tempWeightings[l];
          }
          size_t idx = std::distance(tempDistPerObst.begin(), std::min_element(tempDistPerObst.begin(), tempDistPerObst.end()));
          weightedMinDistPerState[j] = tempDistPerObst[idx];
          unweightedMinDistPerState[j] = unweightedPathDist[idx][i][j];
        }
        size_t idx = std::distance(weightedMinDistPerState.begin(), std::min_element(weightedMinDistPerState.begin(), weightedMinDistPerState.end()));
        result[i] = weightedMinDistPerState[idx];
        result[i + pathN] = unweightedMinDistPerState[idx];
      }
      return result;
    }

    /**
    adds all curvatures in a path together in order to compare it to other paths
    */
    double PathEvaluator::PathStraightness(const std::vector<MukState> path, const std::vector<MukState> goalStates) const
    {
      std::vector<double> Curvatures = computeCurvatures(path);
      double curvaturesCounter(0);
      for (size_t i(0); i < Curvatures.size(); ++i)
        curvaturesCounter += Curvatures[i];
      return curvaturesCounter;
    }

    /**
    calculates the angle between the ideal angle of arrival and the real angle of arrival
    */
    double PathEvaluator::GoalAngleDifference(const MukState& pathEndState, const std::vector<MukState> goalStates) const
    {
      if (goalStates.empty())
        return std::numeric_limits<double>::quiet_NaN();
      double minDist = maxDist;
      size_t idx = 0;
      for (size_t i(0); i < goalStates.size(); ++i)
      {
        double d = (goalStates[i].coords - pathEndState.coords).squaredNorm();
        if (d < minDist)
        {
          minDist = d;
          idx = i;
        }
      }
      return wideAngle(goalStates[idx].tangent, pathEndState.tangent)*360/M_PI;
    }

    /**
    Analyses the air-to-bone-ratio in the surroundings along the path and recognizes airholes along the path.
    Also calculates the bone density along the path but doesn't return it right now. Further testing is needed to see what is important
    */
    double PathEvaluator::TextureSpecifics(const std::vector<MukState> path, const itk::SmartPointer<itk::Image<unsigned short, 3U>> ctImage, const double r, const std::vector<double> holeVar) const
    {
      // determining the transformation of 1 unit in every direction in order to get the stepsize for sweep over the radius of the robot
      itk::Point<double> p1;
      itk::Point<double> p2;
      itk::Index<3U> i1 = { 1,1,1 };
      itk::Index<3U> i2 = { 2,2,2 };
      ctImage->TransformIndexToPhysicalPoint(i1, p1);
      ctImage->TransformIndexToPhysicalPoint(i2, p2);
      p2 = p2 - p1;
      double transformationValue = std::min(p2.Begin()[0], p2.Begin()[1]);
      transformationValue = std::min(transformationValue, p2.Begin()[2]);
      int transformationFactor = ceil(1 / transformationValue);
      // AirtoBoneRatio of the path
      double pathAirtoBoneRatio(0);
      // Mean density of the bone along the path
      double pathDensity(0);
      bool noHoles(true);
      // holeStart will be set to the first point thats considered 'air'
      Vec3d holeStart = { 0,0,0 };
      for (size_t i(0); i < path.size(); ++i)
      {
        // counts the pixels around the current state that are under the threshold
        double airCounter(0);
        // mean density of the bone around the current state
        double circleDensity(0);
        // 16 points around the robot are enough because the circle is small
        std::vector<Vec3d> samples(16);
        // saves the already used indices because some will be used more then once
        std::vector<itk::Index<3U>> usedIndices;
        // with the calculated transformationFactor it's made sure that no Indices in the radius are missed
        for (size_t j(0); (int)j < (transformationFactor*r)+1; ++j)
        {
          // circles around the robot with increasing radii acording the stepsize calculated with the transformationFactor
          circleAroundLine(path[i].coords, path[i].tangent, j / (double)transformationFactor, samples);
          for (size_t k(0); k < samples.size(); ++k)
          {
            double sampleCoords[3] = { samples[k].x(), samples[k].y(), samples[k].z() };
            itk::Point<double> sample = sampleCoords;
            itk::Index<3U> sampleIndex;
            ctImage->TransformPhysicalPointToIndex(sample, sampleIndex);
            bool used = false;
            // check if index was used before
            for (size_t l(0); l < usedIndices.size(); ++l)
            {
              if (usedIndices[l] == sampleIndex)
              {
                used = true;
                break;
              }
            }
            if (!used)
            {
              // if not used yet, increase the airCounter if the pixelValue is under the defined threshhold 
              // and add the pixelValue to the mean density of the radius around the current state
              short pixelValue = ctImage->GetPixel(sampleIndex);
              if (pixelValue <= holeVar[0])
                ++airCounter;
              circleDensity += pixelValue;
              usedIndices.push_back(sampleIndex);
            }
          }
        }
        // When there has not been a hole and the current ratio of air to bone is over the threshold, mark the current State as the start of an airhole
        if (holeStart.isZero() && airCounter / usedIndices.size() >= holeVar[1])
          holeStart = path[i].coords;
        // When in an hole and the current ratio of air to bone is unter the threshold, mark the airhole as over and set the start back to zero
        if (!holeStart.isZero() && airCounter / usedIndices.size() <= holeVar[2])
          holeStart = { 0,0,0 };
        // When in an airhole and the distance from the start of the hole to the current point of the hole is over the threshold
        // break all calculations and return -1 indicating that the path can't be used
        if (!holeStart.isZero() && (holeStart - path[i].coords).squaredNorm() > holeVar[3])
        {
          noHoles = false;
          break;
        }
        pathAirtoBoneRatio += airCounter / usedIndices.size();
        circleDensity /= usedIndices.size(); // circle Density can be used for local deviation
        pathDensity += circleDensity;
      }
      // an airhole was found so the path can't be used
      if (!noHoles)
        return -1;
      pathAirtoBoneRatio /= (double)path.size();
      pathDensity /= (double)path.size();
      //return pathDensity; // alternatively the mean density of the bone along the path can be returned
      return pathAirtoBoneRatio;
    }
  }
}