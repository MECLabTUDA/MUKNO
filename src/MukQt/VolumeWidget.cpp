#include "private/muk.pch"
#include "VolumeWidget.h"

#include <QVTKWidget.h>
#include <QFiledialog.h>
#include <QtWidgets>

namespace gris
{
namespace muk
{
  /**
  */
  VolumeWidget::VolumeWidget(QWidget* parent)
    : QWidget(parent)
  {
    mpLayout = new QGridLayout(this);
    mpLayout->setContentsMargins(0, 0, 0, 0); // makes sure the Widget takes space of the whole parent widget
    mpWidget = new QVTKWidget(this);
    {
      mpWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }
    mpLayout->addWidget(mpWidget, 0, 0, 1, 2);
    
    mpToggleFullScreenButton = new QPushButton(this);
    {
      mpToggleFullScreenButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
      mpToggleFullScreenButton->setText("Toggle Fullscreen");
    }
    mpLayout->addWidget(mpToggleFullScreenButton, 1, 0);
    mpScreenShot = new QPushButton(this);
    {
      mpScreenShot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
      mpScreenShot->setText("ScreenShot");
    }
    mpLayout->addWidget(mpScreenShot, 1,1);
    mpWindow3D = std::make_unique<VtkWindow>(mpWidget->GetRenderWindow());
  }

  /**
  */
  VolumeWidget::~VolumeWidget()
  {
  }
  
  /**
  */
  void VolumeWidget::hide(bool b)
  {
    if (b)
    {
      mpWidget->hide();
      mpToggleFullScreenButton->hide();
      mpScreenShot->hide();
    }
    else
    {
      mpWidget->show();
      mpToggleFullScreenButton->show();
      mpScreenShot->show();
    }
  }

  /**
  */
  void VolumeWidget::takeScreenShot() const
  {
    QString fileName = QFileDialog::getSaveFileName(nullptr, "Save ScreenShot", "D:/");
    if (fileName.isEmpty())
      return;

    mpWindow3D->screenShot(fileName.toLocal8Bit().constData());
  }
}
}
