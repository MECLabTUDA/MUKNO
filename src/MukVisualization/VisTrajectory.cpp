#include "private/muk.pch"
#include "VisTrajectory.h"
#include "ArrowTrajectory.h"

#include <vtkArrowSource.h>
#include <vtkAppendPolyData.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

namespace
{
  const double arbitrary[3] = { vtkMath::Random(-10, 10), vtkMath::Random(-10, 10), vtkMath::Random(-10, 10) };
}

namespace gris
{
namespace muk
{
  /**
  */
  VisTrajectory::VisTrajectory(const std::string& name)
    : VisAbstractObject(name)
  {
  }

  /**
  */
  VisTrajectory::~VisTrajectory()
  {
  }

  /**
  */
  void VisTrajectory::setData(const MukPath& path)
  {
    mTrajectory = path;
  }

  /**
  */
  void VisTrajectory::update()
  {
    const size_t N = mTrajectory.getPath().size();
    if (N == 0)
      return;
    if (N<2)
    {
      // at least 2 states are necessary
      LOG_LINE << "Warning <<" << __FUNCTION__ << ">>: At least 2 states necessary to display" << getName();
      return;
    }
    std::vector<vtkSmartPointer<vtkPolyData>> arrows(N-1);
    auto appendFilter = make_vtk<vtkAppendPolyData>();
    for (size_t i(0); i<N-1; ++i)
    {
      auto& state = mTrajectory.getPath()[i];
      auto& nextState = mTrajectory.getPath()[i+1];
      const auto arrow = Arrow::create(nextState.coords, nextState.tangent);
      arrows[i] = make_vtk<vtkPolyData>();
      arrows[i]->DeepCopy(arrow.getData());
      appendFilter->AddInputData(arrows[i]);
    }
    appendFilter->Update();
    mpData = appendFilter->GetOutput();
    mpMapper->SetInputData(mpData);
    mpMapper->Modified();
  }

}
}