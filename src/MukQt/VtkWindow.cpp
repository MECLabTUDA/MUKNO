#include "private/muk.pch"
#include "MukQt/VtkWindow.h"

#include "MukCommon/muk_common.h"

#include "MukVisualization/muk_colors.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkInteractorStyle.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

namespace gris
{
namespace muk
{
  /**
  */
  VtkWindow::VtkWindow(vtkRenderWindow* pParent)
  {
    mpRenderWindow = pParent;
    // Define viewport ranges
    double xmins[2] = { 0,    0.8 };
    double xmaxs[2] = { 1,    1.0 };
    double ymins[2] = { 0,    0.0 };
    double ymaxs[2] = { 1,    0.2 };

    mpBackgroundRenderer = make_vtk<vtkRenderer>();
    mpBackgroundRenderer->SetBackground(Colors::White);
    mpBackgroundRenderer->SetLayer(0);
        
    mpRenderer = make_vtk<vtkRenderer>();
    mpRenderer->SetLayer(1);
    
    //mpRenderWindow->SetNumberOfLayers(2);
    mpRenderWindow->AddRenderer(mpBackgroundRenderer);
    mpRenderWindow->AddRenderer(mpRenderer);    
    
    mpInteractor = make_vtk<vtkRenderWindowInteractor>();
    mpRenderWindow->SetInteractor(mpInteractor);

    auto axes3D    = make_vtk<vtkAxesActor>();
    mpMarkerWidget = make_vtk<vtkOrientationMarkerWidget>();
    mpMarkerWidget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
    mpMarkerWidget->SetOrientationMarker( axes3D );
    mpMarkerWidget->SetInteractor( mpInteractor );
    mpMarkerWidget->SetEnabled( 1 );
    mpMarkerWidget->InteractiveOff();
    mpMarkerWidget->SetViewport(xmins[1], ymins[1], xmaxs[1], ymaxs[1]);
    axes3D->SetVisibility(0);
  }

  /**
  */
  VtkWindow::~VtkWindow()
  {
  }

  /**
  */
  void VtkWindow::screenShot(const std::string& str)
  {
    // Screenshot  
    auto windowToImageFilter = make_vtk<vtkWindowToImageFilter>();
    windowToImageFilter->SetInput(mpRenderWindow);
    windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
    windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
    windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
    windowToImageFilter->Update();

    auto writer = make_vtk<vtkPNGWriter>();
    writer->SetFileName(str.c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
  }

  /**
  */
  vtkRenderer* VtkWindow::getRenderer()
  {
    return mpRenderer;
  }

  /**
  */
  vtkInteractorStyle* VtkWindow::getInteractorStyle()
  {
    return dynamic_cast<vtkInteractorStyle*>(mpRenderWindow->GetInteractor()->GetInteractorStyle());
  }

  /**
  */
  void VtkWindow::Render()
  {
    mpRenderWindow->Render();
  }

  /**
  */
  void VtkWindow::setFocalPoint(const Vec3d& p)
  {
    mpRenderer->GetActiveCamera()->SetFocalPoint(p.x(), p.y(), p.z());
  }

  /**
  */
  void VtkWindow::setCameraConfiguration(const CameraConfiguration& config)
  {
    auto* camera = mpRenderer->GetActiveCamera();
    const auto& p = config.getPosition();
    camera->SetPosition(p.x(), p.y(), p.z());
    const auto& f = config.getFocalPoint();
    camera->SetFocalPoint(f.x(), f.y(), f.z());
    const auto& v = config.getViewUp();
    camera->SetViewUp(v.x(), v.y(), v.z());      
  }

  /**
  */
  CameraConfiguration VtkWindow::getCameraConfiguration() const
  {
    CameraConfiguration config;
    auto* camera = mpRenderer->GetActiveCamera();
    double* p = camera->GetFocalPoint();
    config.setFocalPoint(Vec3d(p[0], p[1], p[2]));
    p = camera->GetPosition();
    config.setPosition(Vec3d(p[0], p[1], p[2]));
    p = camera->GetViewUp();
    config.setViewUp(Vec3d(p[0], p[1], p[2]));
    return config;
  }

	/**
  */
  void VtkWindow::setInteractorStyle(vtkInteractorStyle* style)
  {
    mpRenderWindow->GetInteractor()->SetInteractorStyle(style);
  }

  /**
  */
  void VtkWindow::setBackgroundColor(const Vec3d& color)
  {
    mpRenderWindow->GetRenderers()->GetFirstRenderer()->SetBackground(color.x(), color.y(), color.z());
  }

  /** \brief

    vtk and its non const getters makes it impossible (not quite) to qualify this function as const
  */
  Vec3d VtkWindow::getBackgroundColor()
  {
    double d[3];
    mpBackgroundRenderer->GetBackground(d);
    return Vec3d(d);
  }

  /**
    // xmin, ymin, xmax, ymax
  */
  void VtkWindow::setViewport(double xmin, double ymin, double xmax, double ymax)
  {
    mpRenderer->SetViewport(xmin, ymin, xmax, ymax);
    mpBackgroundRenderer->SetViewport(xmin, ymin, xmax, ymax);
    mpRenderer->Modified();
    mpBackgroundRenderer->Modified();
  }

  /**
  */
  void VtkWindow::getViewport(double& xmin, double& ymin, double& xmax, double& ymax) const
  {
    double d[4];
    mpRenderer->GetViewport(d);
    xmin = d[0];
    ymin = d[1];
    xmax = d[2];
    ymax = d[3];
  }
}
}
