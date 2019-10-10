#pragma once

#include "BaseController.h"
#include "qevent.h"
#include "VisMukPath.h"

#include <memory>
#include <array>

class QMainWindow;

namespace gris
{
  namespace muk
  {
    class ParetoWidget;
    class SelectionModel;
    class TabSelection;
    class QuickPathAnalysisWindow;

    /** \brief Controls the SelectionModel, the TabSelection, the VisImageOverlay, the AccessCanals and setting those as an obstacle
    */
    class SelectionController : public BaseController
    {
      Q_OBJECT

      signals:

      public:
        SelectionController();
        ~SelectionController();

      public:
        virtual void initialize();
        virtual void setupConnections();
        void finalize();
        void updateSelectionTab();
        
      public:
        void selectPathCollection(); // int index

      public:
        void resetSelection();
        void showSinglePath();
        void showSinglePath(size_t i);
        void showOnlyCollection();
        
        void colorPaths();
        void singlePathSelectionChanged(size_t i);
        void highlightViaSpinBox(size_t i);
        void highlightLargestDistance();
        void highlightStraightesPath();
        void highlightSmallestGoalAngle();
        void highlightShortestPath();
        void highlightLeastThickBone();
        void highlightShortestAirHole();
        void highlightSinglePath(int i);

        void advancedOptionsWindow();
        void advancedOptionsEnlarge();
        void windowClosed();

        void quickPathAnalysis();
        void plotDoubleClicked(QMouseEvent * mouseEvent, QuickPathAnalysisWindow * window);

        void filterPaths();
        void evaluatePaths();
        void updateTabSelectionObstacles();
        void componentWeightingChanged();
        void obstacleWeightingChanged();
        void componentFilterChanged(int ind);
        void obstacleFilterChanged(int ind);

        void updateActiveObstacles();
        void resetTabSelection();
        void recalculateModel();

        void ctFileLoaded();
        void toggleCTOverlay();
        void scrollCTOverlay(bool forward);
        void ctStateChanged(size_t i);
        void displayCTSlice();

        void selectAccessCanal(size_t canalIdx);
        std::vector<bool> cutOffDistanceChanged(size_t i, double cutOffDist);
        void setCanalAsObstacle(size_t i);
        void autoFillCanals();
        void markStatesInPath(std::shared_ptr<VisMukPath> path, std::vector<bool> statesToMark);

        void showParetoFrontWindow();
        void exitParetoFrontWindow();
        void paretoParameterChosen(bool isParam1, const QString& parameterName) const;
        void resetParetoFront();

      public:
        void setDefaultSelectionView();
        void markPathAsSelected(size_t pathIdx, size_t oldCanal = -1);

      private:
        void setCollectionVisibility(const std::string& key, bool visible);
        void addPath(int idx);
        std::vector<std::string> getParetoParams() const;

      private:
        SelectionModel* myModel;
        std::string     mActiveKey;
        bool            mCTFileLoaded = false;
        bool            mOverlayIsActive = false;
        bool            mTabIsLarge = false;
        bool            mTabIsWindow = false;
        bool            mTabNeedsUpdate = false;
        bool            mPathIsColored = false;
        std::vector<bool>   mSetAsObstacle = { false, false, false };
        int             mPositionInPath = 0;  /// Index of the State currently displayed in the CT-Overlay
        std::vector<std::string> mActiveObstacles;
        std::vector<size_t> mfilteredPaths;
        std::unique_ptr<QuickPathAnalysisWindow> mpPlot;

        std::unique_ptr<QMainWindow> mpParetoWindow;
        mutable std::array<std::vector<double>, 2> mDummy;
        ParetoWidget* mpParetoWidget = nullptr;
    };
  }
}
