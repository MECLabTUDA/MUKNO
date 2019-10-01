#include "private/muk.pch"
#include "TabWidgetVisualization.h"
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
  TabWidgetVisualization::TabWidgetVisualization(QWidget* parent)
    : QTabWidget(parent)
  {
    setObjectName("TabWidgetVisualization");
    {
      QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      sizePolicy.setHorizontalStretch(5);
      sizePolicy.setVerticalStretch(0);
      sizePolicy.setHeightForWidth(this->sizePolicy().hasHeightForWidth());
      setSizePolicy(sizePolicy);
      setMinimumSize(QSize(0, 0));
      setBaseSize(QSize(0, 0));
    }

    auto* vtkWindow = new VtkWindow(this);
    vtkWindow->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    addTab(vtkWindow, "3D-view");
	  auto* ctWindow = new CtWindow(this);
	  ctWindow->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	  addTab(ctWindow, "CT 2D-view");
    auto* evalWindow = new EvaluationWindow(this);
    addTab(evalWindow, "Evaluation");
  }

  /**
  **/
  TabWidgetVisualization::~TabWidgetVisualization()
  {
  }

  /**
  */
  EvaluationWindow* TabWidgetVisualization::getEvaluationWindow()
  {
    return findChild<EvaluationWindow*>("EvaluationWindow");
  }

  VtkWindow* TabWidgetVisualization::getVtkWindow()
  {
    return findChild<VtkWindow*>("VtkWindow");
  }

  CtWindow* TabWidgetVisualization::getCtWindow()
  {
	  return findChild<CtWindow*>("CtWindow");
  }

  /**
  */
  void TabWidgetVisualization::setTab(EnWindowType type)
  {
    setCurrentIndex(type);
  }

  /**
  */
  TabWidgetVisualization::EnWindowType TabWidgetVisualization::getTab() const
  {
    return EnWindowType(currentIndex());
  }
}
}
