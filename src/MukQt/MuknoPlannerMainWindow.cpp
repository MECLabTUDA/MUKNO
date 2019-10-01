#include "private/muk.pch"
#include "MuknoPlannerMainWindow.h"

#include "MedicalMultiViewWidget.h"
#include "MukQMenuBar.h"
#include "MukQToolBar.h"
#include "TabImaging.h"
#include "TabNavigation.h"
#include "TabPlanning.h"
#include "TabSelection.h"

#include <qstatusbar.h>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTabWidget>
#include <qlayout.h>
#include <qscrollbar.h>
#include <qimagereader.h>

namespace gris
{
namespace muk
{
  /**
  */
  void MuknoPlannerMainWindow::setupUi(QMainWindow* MainWindow)
  {
    QIcon icon;
    icon.addFile("../resources/icons/muk_icons/gris_logo.png", QSize(), QIcon::Normal, QIcon::Off);
    MainWindow->setWindowIcon(icon);
    MainWindow->setObjectName("MainWindow");

    mMenuBar = new MukQMenuBar(MainWindow);
    mToolBar = new MukQToolBar(MainWindow);
    {
      mToolBar->setupConnections();
      MainWindow->setMenuBar(mMenuBar);
      MainWindow->addToolBar(Qt::TopToolBarArea, mToolBar);
    }

    auto* mainWidget = new QWidget(MainWindow);
    auto  topLayout = new QGridLayout(mainWidget); // horizontal splitterTop with 3 vertical subwidgets/areas
    auto* splitterTop = new QSplitter(mainWidget);
    {
      splitterTop->setOrientation(Qt::Horizontal);
    }
    auto* splitterInfo = new QSplitter(splitterTop);
    {
      splitterInfo->setOrientation(Qt::Vertical);
      mpPropertyWidget = new gris::muk::PropertyWidget(mainWidget);
      mpSceneWidget = new gris::muk::SceneWidget(mainWidget);
      mpTextEditLog = new QTextEdit(mainWidget);
      splitterInfo->addWidget(mpPropertyWidget);
      splitterInfo->addWidget(mpSceneWidget);
      splitterInfo->addWidget(mpTextEditLog);

      splitterTop->addWidget(splitterInfo);
    }

    mpMedMultiViewWidget = new gris::muk::MedicalMultiViewWidget(mainWidget);
    splitterTop->addWidget(mpMedMultiViewWidget);

    mpTabContainer = new QTabWidget(mainWidget);
    {
      mpTabContainer->setLayoutDirection(Qt::LeftToRight);
      mpTabContainer->setMovable(false);
      {
        mpTabImaging    = new gris::muk::TabImaging(mpTabContainer);
        mpTabPlanning   = new gris::muk::TabPlanning(mpTabContainer);
        mpTabSelection  = new gris::muk::TabSelection(mpTabContainer);
        mpTabNavigation = new gris::muk::TabNavigation(mpTabContainer);
        mpTabContainer->insertTab(enImaging, mpTabImaging, QString());
        mpTabContainer->insertTab(enPlanning, mpTabPlanning, QString());
        mpTabContainer->insertTab(enSelection, mpTabSelection, QString());
        mpTabContainer->insertTab(enNavigation, mpTabNavigation, QString());
      }
      splitterTop->addWidget(mpTabContainer);
    }
    topLayout->addWidget(splitterTop, 0, 1, 1, 1);

    MainWindow->setCentralWidget(mainWidget);
    mStatusbar = new QStatusBar(MainWindow);
    MainWindow->setStatusBar(mStatusbar);

    retranslateUi(MainWindow);
    mpTabContainer->setCurrentIndex(1);
    QMetaObject::connectSlotsByName(MainWindow);

  } // setupUi
  /**
  */
  void MuknoPlannerMainWindow::retranslateUi(QMainWindow *MainWindow)
  {
    MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Mukno Planning Tool", 0));

    mpTextEditLog->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
      "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
      "p, li { white-space: pre-wrap; }\n"
      "</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
      "<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", 0));
    mpTabContainer->setTabText(mpTabContainer->indexOf(mpTabImaging), QApplication::translate("MainWindow", "Imaging", 0));
    mpTabContainer->setTabText(mpTabContainer->indexOf(mpTabPlanning), QApplication::translate("MainWindow", "Planning", 0));
    mpTabContainer->setTabText(mpTabContainer->indexOf(mpTabNavigation), QApplication::translate("MainWindow", "Navigation", 0));
    mpTabContainer->setTabText(mpTabContainer->indexOf(mpTabSelection), QApplication::translate("MainWindow", "Selection", 0));
    mpTabPlanning->retranslateUi(MainWindow);
    mpTabNavigation->retranslateUi(MainWindow);
  } // retranslateUi
}
}