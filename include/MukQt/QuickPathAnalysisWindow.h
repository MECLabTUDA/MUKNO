#pragma once

#include "muk_qt_api.h"

#include <QDialog.h>

QT_BEGIN_NAMESPACE
class QCustomPlot;
class QCPItemText;
class QGridLayout;
QT_END_NAMESPACE

namespace gris
{
  namespace muk
  {
    class MUK_QT_API QuickPathAnalysisWindow : public QDialog
    {
      Q_OBJECT

      signals:
        void plotDoubleClicked(QMouseEvent * mouseEvent);
        void pathRequestedClicked(int idx);

      public:
        using ObstaclePeak = std::tuple<std::string,size_t,double>;

      public:
        QuickPathAnalysisWindow(QWidget* parent = nullptr);
        virtual ~QuickPathAnalysisWindow();

      public:
        virtual void open();
        
      public:
        void clearPaths();
        void addDistances(const std::vector<double>& v);
        void addPeaks    (const std::vector<ObstaclePeak>& v);
        void setSafetyDistance(double d);

        QCustomPlot* getPlot() { return mpPlot; }

        void compute();
        void printPlot(const std::string& filename);

      private:
        void adjustDistanceTable(int idx);
        void deletePath(int idx);
        void colorPath (int idx);
        void setPlotWidth(int idx);
        void setFontSize(int idx);
     
      private:
        struct GraphItemContainer;
        std::unique_ptr<GraphItemContainer> mpContainer;
        // plot data
        QCustomPlot* mpPlot;
        
        // plottable data
        double mKappa;

        // qt
        QGridLayout* mpLayout;
        QGridLayout* mpLocalInfoLayout;
    };

  }
}