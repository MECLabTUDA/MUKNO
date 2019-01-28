#pragma once

#include "muk_qt_api.h"

#include "MukCommon/MukVector.h"

#include "MukVisualization/CameraConfiguration.h"

#include <vtkSmartPointer.h>
class vtkRenderer;
class vtkInteractor;
class vtkInteractorStyle;
class vtkOrientationMarkerWidget;

#include <vtkRenderWindow.h>

#include <iostream>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_QT_API VtkWindow
    {
      public:
        VtkWindow(vtkRenderWindow* pParent);
        virtual ~VtkWindow();

      public:
        void Render();

        void setInteractorStyle(vtkInteractorStyle* style);
        void setFocalPoint(const Vec3d& p);        
        void setCameraConfiguration(const CameraConfiguration& config);
        void screenShot(const std::string& filename);
        void  setBackgroundColor(const Vec3d& color);
        Vec3d getBackgroundColor();
        CameraConfiguration getCameraConfiguration() const;


      public:
        vtkRenderer* getRenderer();
        vtkInteractorStyle* getInteractorStyle();

      public:
        void setViewport(double xmin, double ymin, double xmax, double ymax);
        void getViewport(double& xmin, double& ymin, double& xmax, double& ymax) const;

      private:
        vtkRenderWindow*             mpRenderWindow;
        vtkSmartPointer<vtkRenderer> mpRenderer;
        vtkSmartPointer<vtkRenderer> mpBackgroundRenderer;
        vtkSmartPointer<vtkRenderWindowInteractor>    mpInteractor;
        vtkSmartPointer<vtkOrientationMarkerWidget>   mpMarkerWidget;
    };
  }
}
