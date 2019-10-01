#pragma once

#include <QtWidgets/QApplication>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>

//#include "MukQTreeWidget.h"
#include "SceneWidget.h"
#include "PropertyWidget.h"

QT_BEGIN_NAMESPACE
class QGridLayout;
class QSplitter;
class QTabWidget;
class QTextEdit;
QT_END_NAMESPACE

namespace gris
{
  namespace muk
  {
    class MedicalMultiViewWidget;
    class MukQMenuBar;
    class MukQToolBar;
    class TabImaging;
    class TabNavigation;
    class TabPlanning;
    class TabSelection;

    /**
    */
    class MUK_QT_API MuknoPlannerMainWindow
    {
      public:
        void MuknoPlannerMainWindow::setupUi(QMainWindow* MainWindow);

        void retranslateUi(QMainWindow* MainWindow);

      public:
        enum EnModelTabs
        {
          enImaging,
          enPlanning,
          enSelection,
          enNavigation,
        };

      public:
        enum MUK_QT_API EnWindowType
        {
          enTabImaging = 0,
          enTabPlanning,
          enTabSelection,
          enTabEvaluation,
          enTabNavigation
        };

      public:
        MukQMenuBar* mMenuBar;
        MukQToolBar* mToolBar;
        QStatusBar*  mStatusbar;

        PropertyWidget* mpPropertyWidget;
        SceneWidget*    mpSceneWidget;
        QTextEdit*      mpTextEditLog;

        MedicalMultiViewWidget* mpMedMultiViewWidget;

        QTabWidget*     mpTabContainer;
        TabImaging*     mpTabImaging;
        TabPlanning*    mpTabPlanning;
        TabSelection*   mpTabSelection;
        TabNavigation*  mpTabNavigation;
    };
  }
}
