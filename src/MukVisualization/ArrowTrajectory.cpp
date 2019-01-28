#include "private/muk.pch"
#include "ArrowTrajectory.h"

#include "MukCommon/muk_common.h"

#include <vtkAppendPolyData.h>
#include <vtkArrowSource.h>
#include <vtkMath.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

namespace gris
{
namespace muk
{
  /**
  */
  Arrow::Arrow()
  {
    mpArrow = make_vtk<vtkPolyData>();
  }

  /**
  */
  Arrow Arrow::create(const Vec3d& origin, const Vec3d& direction, double length)
  {
    auto arrowSource = make_vtk<vtkArrowSource>();    
    // Compute a basis
    double normalizedX[3] = {direction.x(), direction.y(), direction.z()};
    double normalizedY[3];
    double normalizedZ[3];

    double startPoint[] = { origin.x(), origin.y(), origin.z() };
    double endPoint[]   = { origin.x() + direction.x(), origin.y() + direction.y(), origin.z() + direction.z() };
    
    // The Z axis is an arbitrary vector cross X
    double arbitrary[3];
    arbitrary[0] = vtkMath::Random(-10,10);
    arbitrary[1] = vtkMath::Random(-10,10);
    arbitrary[2] = vtkMath::Random(-10,10);
    vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
    vtkMath::Normalize(normalizedZ);

    // The Y axis is Z cross X
    vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
    auto matrix = make_vtk<vtkMatrix4x4>();

    // Create the direction cosine matrix
    matrix->Identity();
    for (unsigned int i = 0; i < 3; i++)
    {
      matrix->SetElement(i, 0, normalizedX[i]);
      matrix->SetElement(i, 1, normalizedY[i]);
      matrix->SetElement(i, 2, normalizedZ[i]);
    }    

    // Apply the transforms
    auto transform = make_vtk<vtkTransform>();
    transform->Translate(startPoint);
    transform->Concatenate(matrix);
    transform->Scale(length, length, length);

    // Transform the polydata
    auto transformPD = make_vtk<vtkTransformPolyDataFilter>();
    transformPD->SetTransform(transform);
    transformPD->SetInputConnection(arrowSource->GetOutputPort());
    transformPD->Update();

    auto result = Arrow();
    result.mpArrow = transformPD->GetOutput();
    return result;
  }

  /**
  */
  ArrowTrajectory::ArrowTrajectory()    
  { 
    mpData   = make_vtk<vtkPolyData>();
    mpMapper = make_vtk<vtkPolyDataMapper>();
    mpActor  = make_vtk<vtkActor>();
    
    mpMapper->SetInputData(mpData);
    mpActor->SetMapper(mpMapper);    
  }

  /**
  */
  void ArrowTrajectory::clear()
  {
    mArrows.clear();
    mpData->Initialize();
    mpMapper->Modified();
  }

  /**
  */
  void ArrowTrajectory::transform()
  {
    auto appendFilter = make_vtk<vtkAppendPolyData>();
    for (auto arrow : mArrows)
    {
      appendFilter->AddInputData(arrow.getData());
      const int N = arrow.getData()->GetNumberOfPoints();
    }
    if (mArrows.size() > 0)
    {      
      appendFilter->Update();
      mpData = appendFilter->GetOutput();
      mpMapper->SetInputData(mpData);
    }
  }

  size_t ArrowTrajectory::size()
  {
    return mpData->GetNumberOfPoints();
  }

  /**
  */
  void ArrowTrajectory::setColor(const Vec3d& color)
  {    
    mpActor->GetProperty()->SetColor(color.x(), color.y(), color.z());
    mpActor->GetProperty()->SetLineWidth(3.0);
  }

}
}