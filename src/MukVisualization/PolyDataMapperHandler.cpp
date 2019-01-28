#include "private/muk.pch"
#include "PolyDataMapperHandler.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkDoubleArray.h>
#include <vtkPointdata.h>

#include <vtkTubeFilter.h>

namespace gris
{
namespace muk
{
  /**
  */
  void PolyDataMapperHandler::clearTopology(vtkPolyDataMapper* mapper)
  {
  }


  void PolyDataMapperHandler::addTube(vtkPolyDataMapper* mapper, const double radius, const size_t numberOfSlides)
  {    
    vtkPolyData* data = mapper->GetInput();
    if (nullptr == data)
    {
      throw MUK_EXCEPTION_SIMPLE("Empty polyData-input");
    }
    auto tube = make_vtk<vtkTubeFilter>();
    tube->SetInputData(data);
    tube->SetNumberOfSides(static_cast<int>(numberOfSlides));
    tube->SetRadius(radius);    
    mapper->SetInputConnection(tube->GetOutputPort());
  }


  void PolyDataMapperHandler::setColor(vtkPolyDataMapper* mapper, const std::vector<Vec3d>& colors)
  {
    vtkPolyData* data = mapper->GetInput();
    if (nullptr == data)
      return;
    auto colorArray = make_vtk<vtkUnsignedCharArray>();   
    colorArray->SetName("Colors");
    colorArray->SetNumberOfComponents(3);
    const size_t N = data->GetNumberOfLines();
    for (size_t i(0); i < N; ++i)
    {
      colorArray->InsertNextTuple3(colors[i].x(), colors[i].y(), colors[i].z());
    }
    data->GetPointData()->AddArray(colorArray);
    mapper->SetScalarModeToUsePointFieldData();
    mapper->SetColorModeToDefault();
    mapper->SelectColorArray("Colors");
  }

  void PolyDataMapperHandler::setColor(vtkPolyDataMapper* mapper, const double* color)
  {
    vtkPolyData* data = mapper->GetInput();
    if (nullptr == data)
      return;
    auto colorArray = make_vtk<vtkUnsignedCharArray>();
    colorArray->SetName("Colors");
    colorArray->SetNumberOfComponents(3);
    for (int i(0); i < data->GetNumberOfPoints(); ++i)
    {
      colorArray->InsertNextTuple3(color[0], color[1], color[2]);
    }    
    data->GetPointData()->AddArray(colorArray);
    mapper->SetScalarModeToUsePointFieldData();
    mapper->SetColorModeToDefault();
    mapper->SelectColorArray("Colors");
  }


}
}
