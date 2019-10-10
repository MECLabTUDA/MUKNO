#include "private/muk.pch"
#include "PolyDataHandler.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPolyLine.h>
#include <vtkCellArray.h>
#include <vtkPointData.h>

namespace gris
{
namespace muk
{

  /**
  */
  size_t PolyDataHandler::size() const
  {
    return mpData->GetNumberOfPoints();
  }

  /**
  */
  VecN3d PolyDataHandler::getPoints() const 
  { 
    const vtkIdType N = mpData->GetNumberOfPoints();
    VecN3d v(N);
    auto* p = mpData->GetPoints();
    double d[3];
    for (vtkIdType i(0); i < N; ++i)
    {
      p->GetPoint(i, d);
      v[i] = Vec3d(d[0], d[1], d[2]);
    }
    return v;
  }

  /**
  */
  void PolyDataHandler::getPoints(VecN3d& v) const
  {
    v = getPoints();
  }

  /**
  */
  Vec3d PolyDataHandler::getPoint(size_t idx) const
  {
    const size_t N = mpData->GetNumberOfPoints();
    if (idx >= N)
    {
      std::stringstream ss;
      ss << "PolyDataHandler::getPoint -> index out of bounds: " << idx << " >= " << N;
      throw std::exception(ss.str().c_str());
    }    
    double d[3];
    mpData->GetPoint( static_cast<vtkIdType>( idx ) , d);
    return Vec3d(d[0], d[1], d[2]);
  }
  /**
  */
  void PolyDataHandler::getPoint(Vec3d& v, size_t idx) const
  {
    v = getPoint(idx);
  }

  /**
  */
  void PolyDataHandler::clearTopology(vtkPolyData* data)
  {
    if (nullptr != data)
    {
      data->SetVerts(nullptr);
      data->SetLines(nullptr);
      data->BuildLinks();
      data->DeleteCells();
      data->RemoveDeletedCells();
    }
  }

  /**
  */
  void PolyDataHandler::addVertices(vtkPolyData* data)
  {
    const vtkIdType N = data->GetNumberOfPoints();
    DefVtk(vtkCellArray, verts);
    vtkIdType pid[1];
    for (vtkIdType i(0); i < N; ++i)
    {
      pid[0] = i;
      verts->InsertNextCell(1,pid);      
    }
    data->SetVerts(verts);
  }

  void PolyDataHandler::addLines(vtkPolyData* data, double lineWidth)
  {
    const vtkIdType N = data->GetNumberOfPoints();
    DefVtk(vtkPolyLine, lines);
    lines->GetPointIds()->SetNumberOfIds(N);
    for (vtkIdType i(0); i < N; ++i)
    {
      lines->GetPointIds()->SetId(i,i);
    }
    DefVtk(vtkCellArray, linesArray);
    linesArray->InsertNextCell(lines);
    data->SetLines(linesArray);
  }

  void PolyDataHandler::clearTopology()
  {
    clearTopology(this->mpData);
  }

  void PolyDataHandler::addVertices()
  {
    addVertices(this->mpData);
  }

  void PolyDataHandler::addLines()
  {
    addLines(this->mpData);
  }



}
}
