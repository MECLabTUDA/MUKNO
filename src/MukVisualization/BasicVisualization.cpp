#include "private/muk.pch"
#include "BasicVisualization.h"

#include "MukCommon/muk_common.h"

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>



namespace gris
{
namespace muk
{
  /**
  */
  BasicVisualization::BasicVisualization()
  {
    mpRenderer   = make_vtk<vtkRenderer>();
    mpWindow     = make_vtk<vtkRenderWindow>();
    mpWindow->AddRenderer(mpRenderer);
    auto interactor = make_vtk<vtkRenderWindowInteractor>();
    mpWindow->SetInteractor(interactor);
    mpWindow->GetInteractor()->SetInteractorStyle(make_vtk<vtkInteractorStyleTrackballCamera>());


    auto axes = make_vtk<vtkAxesActor>();
    mpAxesOrientation = make_vtk<vtkOrientationMarkerWidget>();
    mpAxesOrientation->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
    mpAxesOrientation->SetOrientationMarker( axes );
    mpAxesOrientation->SetInteractor( interactor );
    mpAxesOrientation->SetViewport( 0.0, 0.0, 0.2, 0.2 );
    mpAxesOrientation->SetEnabled( 1 );
    //mpAxesOrientation->InteractiveOn();

    mpRenderer->SetBackground(1.0, 1.0, 1.0); // white
  }

  /**
  */
  BasicVisualization::~BasicVisualization()
  {
  }

  /**
  */
  vtkRenderer* BasicVisualization::getRenderer()
  {
    return mpRenderer;
  }

  /**
  */
  vtkRenderWindow* BasicVisualization::getWindow()
  {
    return mpWindow;
  }

  /**
  */
  void BasicVisualization::start()
  {
    mpWindow->GetInteractor()->Start();
  }

  /** \brief Creates actor + mapper for the poly data and adds it to the renderer. Returns the id.
  */
  size_t BasicVisualization::addObject(vtkPolyData* pData)
  {
    auto mapper = make_vtk<vtkPolyDataMapper>();
    auto actor = make_vtk<vtkActor>();
    mapper->SetInputData(pData);
    mapper->Modified();
    actor->SetMapper(mapper);
    actor->Modified();
    mpRenderer->AddActor(actor);
    auto tuple = std::make_tuple(actor, mapper, pData);
    mObjects.push_back(tuple);
    return mObjects.size() - 1;
  }

  /**
  */
  size_t BasicVisualization::addProp3D(vtkProp3D* pData)
  {
    mpRenderer->AddViewProp(pData);
    mPropObjects.push_back(pData);
    return mPropObjects.size() - 1;
  }

  /**
  */
  vtkActor* BasicVisualization::objectActor(size_t id)
  {
    return std::get<0>(mObjects[id]);
  }

  /**
  */
  vtkPolyDataMapper* BasicVisualization::objectMapper(size_t id)
  {
    return std::get<1>(mObjects[id]);
  }

  /**
  */
  vtkPolyData* BasicVisualization::objectData(size_t id)
  {
    return std::get<2>(mObjects[id]);
  }

  /**
  */
  void BasicVisualization::clearObjects()
  {
    for (auto& obj : mObjects)
    {
      auto* actor = std::get<0>(obj).GetPointer();
      mpRenderer->RemoveActor(actor);
    }
    mObjects.clear();
  }

  /**
  */
  void BasicVisualization::clearObject(size_t idx)
  {
    if (idx>=mObjects.size())
      return;
    auto actor = std::get<0>(mObjects[idx]);
    mpRenderer->RemoveActor(actor);
    mObjects.erase(mObjects.begin()+idx);
  }

  /** \brief 
  */
  void BasicVisualization::setColor(size_t id, const Vec3d& color)
  {
    std::get<0>(mObjects[id])->GetProperty()->SetColor(color.x() ,color.y(), color.z());
  }

  /** \brief 
  */
  void BasicVisualization::setOpacity(size_t id, double val)
  {
    std::get<0>(mObjects[id])->GetProperty()->SetOpacity(val);
  }

  /**
  */
  vtkActor* BasicVisualization::getActor(size_t idx)
  {
    return std::get<0>(mObjects[idx]).GetPointer(); 
  }

  /**
  */
  vtkPolyDataMapper* BasicVisualization::getMapper(size_t idx)
  {
    return std::get<1>(mObjects[idx]).GetPointer(); 
  }

}
}