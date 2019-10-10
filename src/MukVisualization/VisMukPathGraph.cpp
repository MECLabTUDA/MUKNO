#include "private/muk.pch"
#include "VisMukPathGraph.h"
#include "muk_colors.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkLine.h>

#include <boost/format.hpp>

namespace
{
  using gris::Vec3d;
  using namespace gris::muk;

  void traverse_r(const MukNode& node, vtkIdType nodeID, vtkPoints& points, vtkCellArray& lines, vtkCellArray& vertices)
  {
    for (auto& p : node.children)
    {
      vtkIdType pid  = points.InsertNextPoint(p.mState.coords.data());
      vertices.InsertNextCell(1, &pid);
      auto line = make_vtk<vtkLine>();
      line->GetPointIds()->SetId(0, nodeID);
      line->GetPointIds()->SetId(1, pid);
      lines.InsertNextCell(line);
      traverse_r(p, pid, points, lines, vertices);
    }
  }
}

namespace gris
{
  namespace muk
  {
    /**
    */
    VisMukPathGraph::VisMukPathGraph(const std::string& key)
      : VisAbstractObject(key)
      , mPointSize(2.0)
    {
      setDefaultColor(Colors::Melon);
      setColors(mDefaultColor);
      mpActor->GetProperty()->SetPointSize(mPointSize);

      declareProperty<double>("PointSize"
        , [&] (const double d) { this->setPointSize(d); }
        , [&] () { return this->getPointSize(); });
    }

    /**
    */
    VisMukPathGraph::~VisMukPathGraph()
    {
    }

    /**
    */
    void VisMukPathGraph::setPointSize(double d)
    {
      mPointSize = d;
      mpActor->GetProperty()->SetPointSize(mPointSize);
      mpActor->Modified();
    }

    /**
    */
    void VisMukPathGraph::setData(const MukPathGraph& graph)
    {
      auto data = make_vtk<vtkPolyData>();
      auto points = make_vtk<vtkPoints>();
      auto vertices = make_vtk<vtkCellArray>();
      auto lines = make_vtk<vtkCellArray>();

      const size_t N_Roots = graph.numRoots();
      const size_t N = graph.numNodes();

      std::vector<Vec3d> colors;
      colors.reserve(N);
      for(size_t i(0); i<N_Roots; ++i)
      {
        const auto N = points->GetNumberOfPoints();
        auto id = points->InsertNextPoint(graph.getRoot(i).mState.coords.data());
        vertices->InsertNextCell(1, &id);
        traverse_r(graph.getRoot(i), id, *points, *lines, *vertices);
        const auto M = points->GetNumberOfPoints();
        auto color = i < graph.getSizeStart() ? Vec3d(Colors::Green) : Vec3d(Colors::Blue);
        for(int j(0); j<M-N; ++j)
          colors.push_back(color);
      }
      data->SetPoints(points);
      data->SetLines(lines);
      data->SetVerts(vertices);
      setData(data);
      setColors(colors);
    }
  }
}
