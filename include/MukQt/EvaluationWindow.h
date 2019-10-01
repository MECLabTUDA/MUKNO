#pragma once

#include "muk_qt_api.h"

#include <qwidget.h>
#include <qlayout.h>

#include <memory>

namespace gris
{
  namespace muk
  {

    class MUK_QT_API EvaluationWindow : public QWidget
    {
      Q_OBJECT 

      public:
        EvaluationWindow(QWidget* parent = nullptr);
        ~EvaluationWindow();

       public slots:
        void showNextPlot();

      public:
        void addWidget(QWidget* w, int row, int column, int rowSpan, int colSpan);
        void addPlot(std::shared_ptr<QWidget> w);
        void clearPlots();

      private:
        int mPlotIdx;
        std::vector<std::shared_ptr<QWidget>> mPlots;
        QGridLayout* mpLayout;
    };

  }
}