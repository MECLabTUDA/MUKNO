#include "private/muk.pch"
#include "visualization_tools.h"

#include "ArrowTrajectory.h"
#include "muk_colors.h"
#include "PolyDataHandler.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"
#include "MukCommon/MukPathGraph.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkArrowSource.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
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
    MUK_VIS_API vtkSmartPointer<vtkRenderer> create3DCoordinateAxis()
    {
      auto l_InsertLineFromOrigin = [&] (vtkSmartPointer<vtkPolyData> data, double x, double y, double z)
      {
        auto points = make_vtk<vtkPoints>();
        auto line = make_vtk<vtkPolyLine>();
        auto linesArray = make_vtk<vtkCellArray>();
        points->InsertNextPoint(0, 0, 0);
        points->InsertNextPoint(x, y, z);
        line->GetPointIds()->SetNumberOfIds(2);
        line->GetPointIds()->SetId(0, 0);
        linesArray->InsertNextCell(line);
        data->SetPoints(points);
        data->SetLines(linesArray);
      };

      auto xAxis = make_vtk<vtkPolyData>();
      l_InsertLineFromOrigin(xAxis, 1, 0, 0);
      auto yAxis = make_vtk<vtkPolyData>();
      l_InsertLineFromOrigin(xAxis, 0, 1, 0);
      auto zAxis = make_vtk<vtkPolyData>();
      l_InsertLineFromOrigin(xAxis, 0, 0, 1);

      auto xmapper = make_vtk<vtkPolyDataMapper>();
      auto xactor = make_vtk<vtkActor>();
      xmapper->SetInputData(xAxis);
      xactor->SetMapper(xmapper);
      xactor->GetProperty()->SetColor(Colors::Red);

      auto ymapper = make_vtk<vtkPolyDataMapper>();
      auto yactor = make_vtk<vtkActor>();
      ymapper->SetInputData(yAxis);
      yactor->SetMapper(ymapper);
      yactor->GetProperty()->SetColor(Colors::Green);

      auto zmapper = make_vtk<vtkPolyDataMapper>();
      auto zactor = make_vtk<vtkActor>();
      zmapper->SetInputData(zAxis);
      zactor->SetMapper(zmapper);
      zactor->GetProperty()->SetColor(Colors::Blue);

      auto renderer = make_vtk<vtkRenderer>();
      renderer->AddActor(xactor);
      renderer->AddActor(yactor);
      renderer->AddActor(zactor);
      return renderer;
    }

    /**
    */
    vtkSmartPointer<vtkPolyData> Transformer::createLineSegments(const std::vector<Vec3d>& v)
    {
      const size_t N = v.size();
      auto points = make_vtk<vtkPoints>();
      auto lines = make_vtk<vtkPolyLine>();
      lines->GetPointIds()->SetNumberOfIds(N);      
      for (size_t i(0); i<N; ++i)
      {
        points->InsertNextPoint(v[i].data());
        lines->GetPointIds()->SetId(i, i);
      }
      auto result = make_vtk<vtkPolyData>();
      result->SetPoints(points);
      
      auto linesArray = make_vtk<vtkCellArray>();
      linesArray->InsertNextCell(lines);
      result->SetLines(linesArray);
      return result;
    }

    /**
    */
    vtkSmartPointer<vtkPolyData> Transformer::createPoints(const std::vector<Vec3d>& v)
    {
      const size_t N = v.size();
      auto points = make_vtk<vtkPoints>();
      auto vertices = make_vtk<vtkCellArray>();
      vtkIdType pid[1];
      for (size_t i(0); i<N; ++i)
      {
        pid[0] = points->InsertNextPoint(v[i].data());
        vertices->InsertNextCell(1,pid);
      }
      auto result = make_vtk<vtkPolyData>();
      result->SetPoints(points);
      result->SetVerts(vertices);
      return result;
    }

    /**
    */
    void createVerts(vtkPolyData* input)
    {      
      auto points = make_vtk<vtkPoints>();
      auto vertices = make_vtk<vtkCellArray>();
      vtkIdType pid[1];
      const size_t N = input->GetNumberOfPoints();
      for (size_t i(0); i<N; ++i)
      {
        double* p = input->GetPoint(i);
        pid[0] = points->InsertNextPoint(p[0], p[1], p[2]);
        vertices->InsertNextCell(1,pid);
      }
      auto result = make_vtk<vtkPolyData>();
      result->SetPoints(points);
      result->SetVerts(vertices);      
      input->DeepCopy(result);
    }

    /**
    */
    void createLines(vtkPolyData* input)
    { 
      const size_t N = input->GetNumberOfPoints();
      auto points = make_vtk<vtkPoints>();
      auto lines = make_vtk<vtkPolyLine>();
      auto vertices = make_vtk<vtkCellArray>();
      vtkIdType pid[1];
      lines->GetPointIds()->SetNumberOfIds(N);
      for (size_t i(0); i<N; ++i)
      {
        double* p = input->GetPoint(i);
        pid[0] = points->InsertNextPoint(p[0], p[1], p[2]);
        vertices->InsertNextCell(1,pid);
        lines->GetPointIds()->SetId(i, i);        
      }
      auto result = make_vtk<vtkPolyData>();
      result->SetPoints(points);      
      result->SetVerts(vertices);     
      auto linesArray = make_vtk<vtkCellArray>();
      linesArray->InsertNextCell(lines);
      result->SetLines(linesArray);
      input->DeepCopy(result);
    }

    /**
    */
    void setToVertices(vtkPolyData* pData)
    {
      // see http://www.itk.org/Wiki/VTK/Examples/Cxx/Broken/PolyData/DeleteCells
      // Tell the polydata to build 'upward' links from points to cells.      
      //pData->BuildLinks();
      //// Mark a cell as deleted.
      //LOG_LINE << "test2\n";
      //for (int i(0); i<pData->GetNumberOfCells(); ++i)
      //  pData->DeleteCell(i);
      //// Remove the marked cell.
      //LOG_LINE << "test3\n";
      createVerts(pData);
    }

    /**
    */
    void setToLines(vtkPolyData* pData)
    {
      createLines(pData);
    }

    /**
      example : lower left corner front = llcf
                upper right corner back = urcb

      ---> x
      ^
      |  y
      |
        z
       /
      /

        x -----x      ulcb ---- urcb
       /      /|       /         / |
      x ---- x |    ulcf ---- urcf |
      |      | x      |         |  lrcb
      |      |/       |         |  /
      x ---- x      llcf ---- lrcf
    */
    void createBounds(const IBounds& bounds, vtkPolyData* output)
    {
      auto min = bounds.getMin();
      auto max = bounds.getMax();
      double dx = max.x() - min.x();
      double dy = max.y() - min.y();
      double dz = max.z() - min.z();
      Vec3d llcf = min;
      Vec3d lrcf = Vec3d(max.x(), min.y(), min.z());
      Vec3d ulcf = Vec3d(min.x(), max.y(), min.z());
      Vec3d urcf = Vec3d(max.x(), max.y(), min.z());

      Vec3d llcb = Vec3d(min.x(), min.y(), max.z());
      Vec3d lrcb = Vec3d(max.x(), min.y(), max.z());
      Vec3d ulcb = Vec3d(min.x(), max.y(), max.z());
      Vec3d urcb = max;

      auto points = make_vtk<vtkPoints>();
      {
        points->InsertNextPoint(llcf.x(), llcf.y(), llcf.z());
        points->InsertNextPoint(lrcf.x(), lrcf.y(), lrcf.z());
        points->InsertNextPoint(ulcf.x(), ulcf.y(), ulcf.z());
        points->InsertNextPoint(urcf.x(), urcf.y(), urcf.z());

        points->InsertNextPoint(llcb.x(), llcb.y(), llcb.z());
        points->InsertNextPoint(lrcb.x(), lrcb.y(), lrcb.z());
        points->InsertNextPoint(ulcb.x(), ulcb.y(), ulcb.z());
        points->InsertNextPoint(urcb.x(), urcb.y(), urcb.z());
      }
      output->SetPoints(points);
      PolyDataHandler::addVertices(output);
    }

    /**
    */
    vtkSmartPointer<vtkPolyData> createBounds(const IBounds& bounds)
    {
      auto output = make_vtk<vtkPolyData>();
      createBounds(bounds, output.Get());
      return output;
    }
    
    /**
    */
    vtkSmartPointer<vtkPolyData> Transformer::createArrows(const std::vector<MukState>& v, double arrowLength)
    {
      auto result      = make_vtk<vtkPolyData>();          
      auto dataArrow   = make_vtk<vtkPolyData>();          
      auto arrowSource = make_vtk<vtkArrowSource>();
      {
        auto appendFilter = make_vtk<vtkAppendPolyData>();
        const size_t N = v.size();
        std::vector<vtkSmartPointer<vtkPolyData>> arrows(N);
        for (int idx(0); idx<N; ++idx)
        {
          const auto arrow = Arrow::create(v[idx].coords, v[idx].tangent);
          arrows[idx] = make_vtk<vtkPolyData>();
          arrows[idx]->DeepCopy(arrow.getData());
          appendFilter->AddInputData(arrows[idx]);
        }
        if (N>0)
        {
          appendFilter->Update();
          dataArrow = appendFilter->GetOutput();
        }
      }
      result = dataArrow;
      return result;
    }
  }   
}