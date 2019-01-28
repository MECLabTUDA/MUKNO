#pragma once
#include "muk_qt_api.h"
#include "MukQt/VtkWindow.h"

#include "MukCommon/MukVector.h"

#include "MukVisualization/CameraConfiguration.h"

#include <vtkSmartPointer.h>
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkInteractor;
class vtkInteractorStyle;
class vtkOrientationMarkerWidget;
class QVTKWidget;

#include <QWidget>
#include <QLayout>
#include <QPushButton>

#include <memory>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API VolumeWidget : public QWidget
    {
      public:
        VolumeWidget(QWidget* parent = nullptr);
        virtual ~VolumeWidget();

      public:
        VtkWindow*       getVtkWindow()       { return mpWindow3D.get(); }
        const VtkWindow* getVtkWindow() const { return mpWindow3D.get(); }
        
        QGridLayout*     getLayout()           { return mpLayout; }
        QPushButton*     getFullScreenButton() { return mpToggleFullScreenButton; } 
        QPushButton*     getScreenShotButton() { return mpScreenShot; } 
        void hide(bool b);
        void takeScreenShot() const;

      private:
        std::unique_ptr<VtkWindow>   mpWindow3D;
        QVTKWidget*                  mpWidget;
        QPushButton*                 mpToggleFullScreenButton;
        QPushButton*                 mpScreenShot;
        QGridLayout*                 mpLayout;
    };
  }
}