#include "private/muk.pch"
#include "SliceWidget.h"

#include "MukVisualization/SliceRenderGroup.h"

#include <QVTKWidget.h>
#include <QLabel.h>

// #include <boost/format.hpp>
#include <sstream>

namespace gris
{
namespace muk
{
  /**
  */
  SliceWidget::SliceWidget(QWidget* pParent)
    : mpSliceRenderGroup(std::make_unique<SliceRenderGroup>())
  {
    mpWidget = new QVTKWidget(pParent);
    mpWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    mpLayout = new QGridLayout();
    mpLayout->addWidget(mpWidget, 0,0,1,2);
    mpToggleFullScreenButton = new QPushButton(pParent);
    {
      mpToggleFullScreenButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
      mpToggleFullScreenButton->setText("Toggle Fullscreen");
    }
    mpSliceNumberLabel = new QLabel(pParent);
    {
      mpSliceNumberLabel->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
      mpSliceNumberLabel->setText(" / ");
    }
    mpLayout->addWidget(mpToggleFullScreenButton, 1,0);
    mpLayout->addWidget(mpSliceNumberLabel, 1,1);
    mpSliceRenderGroup->setRenderWindow(mpWidget->GetRenderWindow());
  }
  
  void SliceWidget::hide(bool b)
  {
    mpWidget->setVisible(b);
    mpToggleFullScreenButton->setVisible(b);
    mpSliceNumberLabel->setVisible(b);
  }

  /**
  */
  SliceWidget::~SliceWidget()
  {
  }

  /**
  */
  void SliceWidget::setSliceNumber(int current, int max)
  {
    //auto form = boost::format("% 4i / % 4i") & current & max;
    std::stringstream form;
    form << current << " / " << max;
    mpSliceNumberLabel->setText(form.str().c_str());
  }

  /**
  */
  void SliceWidget::setValue(int x, int y, int z, double v)
  {
    std::stringstream form;
    form << "[" << x << ", " << y << ", " << z << "] = " << v;
    mpSliceNumberLabel->setText(form.str().c_str());
  }
}
}