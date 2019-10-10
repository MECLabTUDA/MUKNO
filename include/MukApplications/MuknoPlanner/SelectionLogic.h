#pragma once

#include "BaseLogic.h"
#include "VisPathCollection.h"
#include "VisMukPath.h"
#include <PlanningLogic.h>
#include <PropertyWidget.h>

#include <MukCommon/MukScene.h>
#include <MukEvaluation/statistics.h>
#include "MukImaging/MukImage.h"

#include "gstd/DynamicProperty.h"

#include <itkImageToVTKImageFilter.h>

#include <qobject.h>
#include <qwidget.h>

#include <memory>

class vtkImplicitPlaneWidget2;
class vtkImplicitPlaneRepresentation;

namespace gris
{
  namespace muk
  {
    class MukQMenuBar;
    class MukQToolBar;
    class MuknoPlannerMainWindow;
    class VisScene;
    class TabSelection;
    class PaintImageCallback;

    /** \brief performs the selection process. Provides functions for visualization of differences between paths and highlighting of specific paths.
    */
    class SelectionLogic : public BaseLogic, public gstd::DynamicProperty
    {
      Q_OBJECT

    public:
      explicit SelectionLogic(std::shared_ptr<MukScene> pScene);

    public:
      void setupConnections(MuknoPlannerMainWindow* pWindow);

    public:
      // brief reset to original Paths, delete all changes to Paths
      void resetSelection();
      void resetPaths();
      void showSinglePath();
      void showSinglePath(size_t i);
      void highlightSinglePath(size_t i);
      void colorPaths();
      void evaluatePaths();
      void categoryWeightingChanged(double d, int i);
      void obstacleWeightingChanged(double d, int i);
      void toggleCTOverlay();
      void scrollCTOverlay(bool forward);
      void ctStateChanged(size_t i);
      void displayCTSlice();

      void propertyChanged();

    private:
      MukScene* mpScene;
      VisScene* mpVisScene;
      TabSelection* mpTabSelection;
      PlanningLogic* mpPlanningLogic;
      QTabWidget* mpTabContainer;
      PropertyWidget* mpPropertyWidget;
      std::vector<double> weightings;
      std::vector<double> holeVariables;
      bool ctOverlayActive = false;
      // Index of the State currently displayed in the CT-Overlay
      size_t positionInPath = 0;
      const double maxDist = std::numeric_limits<double>::infinity();

      vtkSmartPointer<vtkImplicitPlaneWidget2> mpImpPlaneWidget;
      vtkSmartPointer<vtkImplicitPlaneRepresentation> mpPlaneRep;
      vtkSmartPointer<PaintImageCallback> mpCallback;
      itk::ImageToVTKImageFilter<MukImage>::Pointer mpToVtk;
    };
  }
}
