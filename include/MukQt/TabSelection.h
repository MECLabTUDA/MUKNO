#pragma once

#include "muk_qt_api.h"

#include <QWidget>
#include <QLayout>
#include <QtWidgets/QPushButton>
#include <qgraphicsscene.h>

class QSpinBox;

#include <qlabel.h>

namespace gris
{
  namespace muk
  {
    class MukScene;
    class ParameterNet;

    /**
    */
    class MUK_QT_API TabSelection : public QWidget
    {
      Q_OBJECT

      signals:
        void singlePathSelectionChanged(size_t i);

        void componentWeightingChanged();
        void obstacleWeightingChanged();
        void evaluatePathClicked();

        void componentFilterChanged(int ind);
        void obstacleFilterChanged(int ind);
        void filterPathsClicked();

        void ctOverlayClicked();
        void ctOverlayScrolled(bool forward);
        void displayedStateChanged(size_t i);

        void selectLargestDistanceClicked();
        void selectStraightestPathClicked();
        void selectBestAnglePathClicked();
        void selectShortestPathClicked();
        void selectLeastThickBoneClicked();
        void selectShortestAirHoleClicked();

        void showOnlyClicked();
        void colorPathsClicked();
        void resetSelectionClicked();
        void enlargeClicked();
        void windowClicked();

        void indexChosenClicked(size_t i);
        void cutOffDistanceChanged(size_t i, double cutOffDistance);
        void asObstacleClicked(size_t i);
        void fillCanalsClicked();

      public:
        TabSelection(QWidget *parent = nullptr);
        ~TabSelection() = default;

      public:
        void    setLayoutLarge(bool large);
        
        void    updateComponentCount(bool ctFileLoaded, bool tabIsSmall);
        void    updateObstacleCount(bool tabIsSmall, std::vector<std::string> activeObstacleKeys, std::vector<QColor> colors);
        void    updateComponentWeights(std::vector<double> weights);
        void    updateObstacleWeights(std::vector<double> weights);
        void    updateComponentFilter(std::vector<double> sliderValue, std::vector<double> filterValue, std::vector<double> filterMin, std::vector<double> filterMax);
        void    updateObstacleFilter(std::vector<double> sliderValue, std::vector<double> filterValue, std::vector<double> filterMin, std::vector<double> filterMax);
        
        void    resetTabSelection();

        void    toggleCTOverlay(bool on);
        void    toggleAdvancedOptionsWindow(bool on);
        void    toggleAdvancedOptionsEnlarge(bool on);
        void    setSinglePathSelection(size_t i);
        size_t  getSinglePathSelection() const;
        void    updateDisplayState(size_t i, size_t max = 99);
        void    setMaxPathIndex(int idx);

        void    setAccessCanalIndex(int canalIdx, int pathIdx);
        void    checkAccessCanals(std::vector<size_t> chosenIdx);
        void    toggleAsObstacle(bool on, size_t i);
        void    updateCutOffDistance(size_t canalInd, double distance);
        

        double  roundDouble(double value, size_t decimals);

      public:
        std::vector<double> getComponentWeights() const;
        std::vector<double> getComponentFilter() const;
        std::vector<double> getObstacleWeights()  const;
        std::vector<double> getObstacleFilter()  const;
        double getCutOffDistance(size_t canalInd);

      protected:
        virtual void wheelEvent(QWheelEvent *);
        
      private:
        QLabel* mAccessCanals[3];
        ParameterNet* mpComponentNet;
        ParameterNet* mpObstacleNet;
        QSpinBox* mpSelectedPath;
        QSpinBox* mpSelectedState;
        QGridLayout* mpLayoutSmall;
        QGridLayout* mpLayoutLarge;
        QWidgetList mpLayoutLargeCollector;
        QWidgetList mpLayoutSmallCollector;
        
    };
  }
}