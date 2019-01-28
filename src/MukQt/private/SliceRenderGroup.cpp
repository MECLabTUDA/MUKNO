#include "private/muk.pch"
#include "private/SliceRenderGroup.h"

#include "MukCommon/muk_common.h"

#include "MukVisualization/muk_colors.h"

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

namespace gris
{
namespace muk
{
  /**
  */
  SliceRenderGroup::SliceRenderGroup(vtkRenderWindow* pParent)
  {
    mpRenderWindow = pParent;

    mpBackgroundRenderer = make_vtk<vtkRenderer>();
    mpBackgroundRenderer->SetBackground(Colors::White);
    mpBackgroundRenderer->SetLayer(0);

    mpRenderer = make_vtk<vtkRenderer>();
    mpRenderer->SetLayer(1);

    //mpRenderWindow->SetNumberOfLayers(2);
    mpRenderWindow->AddRenderer(mpBackgroundRenderer);
    mpRenderWindow->AddRenderer(mpRenderer);    

    //mpInteractor = make_vtk<vtkRenderWindowInteractor>();
    //mpRenderWindow->SetInteractor(mpInteractor);
  }

  /**
  */
  SliceRenderGroup::~SliceRenderGroup()
  {
  }

  /**
  */
  vtkRenderer* SliceRenderGroup::getRenderer()
  {
    return mpRenderer;
  }

  /**
  */
  /*vtkInteractorStyle* SliceRenderGroup::getInteractorStyle()
  {
    return dynamic_cast<vtkInteractorStyle*>(mpRenderWindow->GetInteractor()->GetInteractorStyle());
  }*/

  /**
  */
  void SliceRenderGroup::render()
  {
    mpRenderWindow->Render();
  }
  
  /**
  */
  /*void SliceRenderGroup::setInteractorStyle(vtkInteractorStyle* style)
  {
    mpRenderWindow->GetInteractor()->SetInteractorStyle(style);
  }*/

  /**
  */
  void SliceRenderGroup::setBackgroundColor(const Vec3d& color)
  {
    mpBackgroundRenderer->SetBackground(color.x(), color.y(), color.z());
  }

  /** \brief

  vtk and its non const getters makes it impossible (not quite) to qualify this function as const
  */
  Vec3d SliceRenderGroup::getBackgroundColor()
  {
    double d[3];
    mpBackgroundRenderer->GetBackground(d);
    return Vec3d(d);
  }

  /**
  // xmin, ymin, xmax, ymax
  */
  void SliceRenderGroup::setViewport(double xmin, double ymin, double xmax, double ymax)
  {
    mpRenderer->SetViewport(xmin, ymin, xmax, ymax);
    mpBackgroundRenderer->SetViewport(xmin, ymin, xmax, ymax);
    mpRenderer->Modified();
    mpBackgroundRenderer->Modified();
  }

  /**
  */
  void SliceRenderGroup::getViewport(double& xmin, double& ymin, double& xmax, double& ymax) const
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