#include "private/muk.pch"
#include "VisualizationWindow.h"
#include "MukQt/VtkWindow.h"
#include "EvaluationWindow.h"
#include "CtWindow.h"

#include "vtkRenderer.h"

#include <qlayout.h>
#include <qtabwidget.h>

namespace gris
{
namespace muk
{  
  /**
  */
  VisualizationWindow::VisualizationWindow(QWidget* parent)
    : QWidget(parent)
  {
    setObjectName("VisualizationWindow");
    {
      QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      sizePolicy.setHorizontalStretch(5);
      sizePolicy.setVerticalStretch(0);
      sizePolicy.setHeightForWidth(this->sizePolicy().hasHeightForWidth());
      setSizePolicy(sizePolicy);
      setMinimumSize(QSize(0, 0));
      setBaseSize(QSize(0, 0));
    }
    auto layout     = new QVBoxLayout(this);
    auto tabWindows = new QTabWidget(this);
    {
      tabWindows->setObjectName("VisualizationTabWidget");
      QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      sizePolicy.setHeightForWidth(tabWindows->sizePolicy().hasHeightForWidth());
      tabWindows->setSizePolicy(sizePolicy);
      tabWindows->setMinimumSize(QSize(0, 0));
      tabWindows->setMaximumSize(QSize(16777215, 16777215));
      tabWindows->setBaseSize(QSize(0, 0));
      tabWindows->setLayoutDirection(Qt::LeftToRight);
      tabWindows->setMovable(false);
    }
    layout->addWidget(tabWindows);

    auto vtkWindow = new VtkWindow(this);
    vtkWindow->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    tabWindows->addTab(vtkWindow, "3D-view");
    auto evalWindow = new EvaluationWindow(this);
    tabWindows->addTab(evalWindow, "Evaluation");    

	  auto ctWindow = new CtWindow();
	  ctWindow->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	  tabWindows->addTab(ctWindow, "CT 2D-view");

	  connect(tabWindows, &QTabWidget::currentChanged, this, &VisualizationWindow::currentTabChanged);
  }

  /**
  **/
  VisualizationWindow::~VisualizationWindow()
  {
  }

  void VisualizationWindow::currentTabChanged(int index)
  {
    enum EnTabIndex
    {
      tab3D = 0,
      tabEval,
      tab2D
    };
  }

  /**
  */
  EvaluationWindow* VisualizationWindow::getEvaluationWindow()
  {
    return findChild<EvaluationWindow*>("EvaluationWindow");
  }

  VtkWindow* VisualizationWindow::getVtkWindow()
  {
    return findChild<VtkWindow*>("VtkWindow");
  }

  CtWindow* VisualizationWindow::getCtWindow()
  {
	return findChild<CtWindow*>("CtWindow");
  }

  /**
  */
  void VisualizationWindow::setTab(EnWindowType type)
  {
    findChild<QTabWidget*>("VisualizationTabWidget")->setCurrentIndex(type);
  }

  /**
  */
  VisualizationWindow::EnWindowType VisualizationWindow::getTab() const
  {
    return EnWindowType(findChild<QTabWidget*>("VisualizationTabWidget")->currentIndex());
  }
}
}
