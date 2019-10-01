#include "private/muk.pch"
#include "SphereInteraction.h"
#include "private/DirectionInteractionStyle.h"

#include "MukCommon/vtk_tools.h"

#include <qlayout.h>
#include <QVTKWidget.h>

#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkAxesActor.h>

#include <vtkMath.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkArrowSource.h>
#include <vtkCamera.h>


namespace gris
{
  namespace muk
  {
    /**
    */
    struct SphereInteraction::ArrowMaker
    {
      ArrowMaker();
      ~ArrowMaker() {}

      void setDirection(const Vec3d& direction);
      Vec3d getDirection() const;

      vtkSmartPointer<vtkArrowSource>    mpArrow;
      vtkSmartPointer<vtkTransform>      mpArrowTransform;
      vtkSmartPointer<vtkTransformPolyDataFilter> mpPolyTransform;
      vtkSmartPointer<vtkMatrix4x4>      mpMatrix;
      vtkSmartPointer<vtkActor>          mpActor;
    };

    /**
    */
    SphereInteraction::ArrowMaker::ArrowMaker()      
      : mpArrow (make_vtk<vtkArrowSource>())
      , mpMatrix (make_vtk<vtkMatrix4x4>())
      , mpArrowTransform (make_vtk<vtkTransform>())
      , mpPolyTransform  (make_vtk<vtkTransformPolyDataFilter>())
      , mpActor (make_vtk<vtkActor>())
    {      
      // Transform the polydata
      mpMatrix->Identity();
      mpArrowTransform->SetMatrix(mpMatrix);
      mpPolyTransform->SetTransform(mpArrowTransform);
      mpPolyTransform->SetInputConnection(mpArrow->GetOutputPort());      
      //Create a mapper and actor for the arrow
      //mpDirection = make_vtk<vtkPolyData>();
      auto mapper = make_vtk<vtkPolyDataMapper>();
      mapper->SetInputConnection(mpPolyTransform->GetOutputPort());
      mpActor->SetMapper(mapper);
      mpActor->GetProperty()->SetColor(1,1,1);
    }

    /**
    */
    void SphereInteraction::ArrowMaker::setDirection(const Vec3d& direction)
    {
      // Compute a basis
      double normalizedX[3] = {direction.x(), direction.y(), direction.z()};
      double normalizedY[3];
      double normalizedZ[3];
      // The Z axis is an arbitrary vector cross X
      const double arbitrary[3] = { vtkMath::Random(-10,10), vtkMath::Random(-10,10), vtkMath::Random(-10,10) };
      vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
      vtkMath::Normalize(normalizedZ);
      // The Y axis is Z cross X
      vtkMath::Cross(normalizedZ, normalizedX, normalizedY);      
      // Create the direction cosine matrix
      mpMatrix->Identity();
      for (unsigned int i = 0; i < 3; i++)
      {
        mpMatrix->SetElement(i, 0, normalizedX[i]);
        mpMatrix->SetElement(i, 1, normalizedY[i]);
        mpMatrix->SetElement(i, 2, normalizedZ[i]);
      }
      mpArrowTransform->SetMatrix(mpMatrix);
      mpActor->GetMapper()->Update();      
    }

    /**
    */
    Vec3d SphereInteraction::ArrowMaker::getDirection() const
    {
      return Vec3d( mpMatrix->GetElement(0,0), mpMatrix->GetElement(1,0), mpMatrix->GetElement(2,0) );
    }

    /**
    */
    SphereInteraction::SphereInteraction(QWidget* widget)
      : QWidget(widget)
      , mCameraDistance(3.0)
      , mpRenderer  (make_vtk<vtkRenderer>())
      , mpArrowMaker( std::make_unique<ArrowMaker>() )
    {
      mpRenderer->AddActor(mpArrowMaker->mpActor);
      this->setMinimumHeight(150);
      this->heightForWidth(1);      
    }

    /**
    */
    SphereInteraction::~SphereInteraction()
    {
    }

    /** \brief builds up a vtkWidget and rendering of a sphere
    */
    void SphereInteraction::initialize()
    {      
      // Define viewport ranges
      double xmins[2] = { 0,    0.0 };
      double xmaxs[2] = { 1,    1.0 };
      double ymins[2] = { 0,    0.0 };
      double ymaxs[2] = { 1,    1.0 };

      auto backgroundRenderer = make_vtk<vtkRenderer>();
      backgroundRenderer->SetBackground(0.3, 0.6, 0.3);
      backgroundRenderer->SetViewport(xmins[0], ymins[0], xmaxs[0], ymaxs[0]);
      backgroundRenderer->SetLayer(0);

      mpRenderer->SetBackground(0.3, 0.6, 0.3);
      mpRenderer->SetLayer(1);
      mpRenderer->SetViewport(xmins[0], ymins[0], xmaxs[0], ymaxs[0]);
      mpRenderer->SetBackground(0.3, 0.6, 0.3); // Background color green

      setDirection(Vec3d(1,0,0));

      mpRenderWindow = make_vtk<vtkRenderWindow>();
      mpRenderWindow->SetNumberOfLayers(3);
      mpRenderWindow->AddRenderer(backgroundRenderer);
      mpRenderWindow->AddRenderer(mpRenderer);

      auto layout = new QVBoxLayout(this);
      auto mVtkWidget    = new QVTKWidget(this);
      mVtkWidget->SetRenderWindow(mpRenderWindow);
      layout->addWidget(mVtkWidget);

      auto interactor3D      = make_vtk<vtkRenderWindowInteractor>();
      mpRenderWindow->SetInteractor(interactor3D);
      mpRenderer->GetActiveCamera()->SetPosition(0,0,mCameraDistance);

      auto axes3D = make_vtk<vtkAxesActor>();
      markerWidget = make_vtk<vtkOrientationMarkerWidget>(); // needs to be member variable
      markerWidget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
      markerWidget->SetOrientationMarker( axes3D );
      markerWidget->SetInteractor( interactor3D );
      markerWidget->SetEnabled( 1 );
      markerWidget->InteractiveOff();      
      markerWidget->SetViewport(xmins[1], ymins[1], xmaxs[1], ymaxs[1]);

      auto mStyle = make_vtk<DirectionInteractionStyle>();      
      interactor3D->SetInteractorStyle(mStyle);
      mStyle->SetCurrentRenderer(mpRenderer);
      mStyle->setInteractor(this);
      
      mpRenderWindow->Render();
      // initialize with random vector (rendering not yet possible)      
      Vec3d tmp (1,1,0);
      tmp.normalize();
      setDirection(tmp);      
    }

    /** \brief updates the visualization of the direction (arrow) und signals change of direction
    */
    void SphereInteraction::setDirection(const Vec3d& direction)
    {
      // update arrow
      mpArrowMaker->setDirection(direction);
      // signal change of direction
      emit directionChanged();
    }

    /** \brief updates the visualization of the direction (arrow) only
      used if user selects a new waypoint
    */
    void SphereInteraction::changeDirection(const Vec3d& direction)
    {
      mpArrowMaker->setDirection(direction);
      Vec3d cameraPosition(direction);
      cameraPosition.normalize();
      cameraPosition *= mCameraDistance;
      mpRenderer->GetActiveCamera()->SetPosition(cameraPosition.x(), cameraPosition.y(), cameraPosition.z());
      mpRenderer->Render();
    }

    /**
    */
    void SphereInteraction::render()
    {
      mpRenderWindow->Render();
    }

    /**
    */
    Vec3d SphereInteraction::getDirection() const
    {
      return mpArrowMaker->getDirection();
    }

  }
}

