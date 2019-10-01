#pragma once

#include "muk_qt_api.h"

#include "MukCommon/MukVector.h"

#include <qwidget.h>
#include <QTabWidget>

namespace gris
{
  namespace muk
  {
    class EvaluationWindow;
    class VtkWindow;
	  class CtWindow;

    /**
    */
    class MUK_QT_API TabWidgetVisualization : public QTabWidget
    {
      Q_OBJECT

      public:
        TabWidgetVisualization(QWidget* parent = nullptr);
        ~TabWidgetVisualization();

      public:
        enum MUK_QT_API EnWindowType
        {
          enView3D = 0,
          enViewSlices,
          enViewEval
        };

      public:
        VtkWindow*        getVtkWindow();
		    CtWindow*         getCtWindow();
    };

  }
}
