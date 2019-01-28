#include "private/muk.pch"
#include "VisBounds.h"
#include "muk_colors.h"

#include "MukCommon/IBounds.h"
#include "MukCommon/MukException.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkPolyLine.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>

namespace gris
{
namespace muk
{
  /**
  */
  VisBounds::VisBounds()
    : VisAbstractObject("Bounds")
    , mMin(0,0,0)
    , mMax(0,0,0)
  {
    setDefaultColor(Colors::Black);
    setColors(mDefaultColor);

    declareProperty<double>("LineWidth",
      [&] (const auto& val) { mpActor->GetProperty()->SetLineWidth(val); },
      [&] ()                { return mpActor->GetProperty()->GetLineWidth(); });
  }

  /**
  */
  VisBounds::~VisBounds()
  {
  }

  /**
  */
  void VisBounds::update()
  {
    auto points = make_vtk<vtkPoints>();
    // add points
    points->InsertNextPoint(mMin.x(), mMin.y(), mMin.z());
    points->InsertNextPoint(mMax.x(), mMin.y(), mMin.z());
    points->InsertNextPoint(mMax.x(), mMax.y(), mMin.z());
    points->InsertNextPoint(mMin.x(), mMax.y(), mMin.z());
        
    points->InsertNextPoint(mMin.x(), mMin.y(), mMax.z());
    points->InsertNextPoint(mMax.x(), mMin.y(), mMax.z());
    points->InsertNextPoint(mMax.x(), mMax.y(), mMax.z());
    points->InsertNextPoint(mMin.x(), mMax.y(), mMax.z());

    auto linesArray = make_vtk<vtkCellArray>();    
    // add lines
    // lower lines
    auto lines = make_vtk<vtkPolyLine>();
    lines->GetPointIds()->SetNumberOfIds(5);
    for  (int i(0); i<4; ++i)
      lines->GetPointIds()->SetId(i,i);
    lines->GetPointIds()->SetId(4,0);
    linesArray->InsertNextCell(lines);
    // upper lines
    lines = make_vtk<vtkPolyLine>();
    lines->GetPointIds()->SetNumberOfIds(5);
    for  (int i(0); i<4; ++i)
      lines->GetPointIds()->SetId(i,i+4);
    lines->GetPointIds()->SetId(4,4);
    linesArray->InsertNextCell(lines);
    for (int i(0); i<4; ++i)
    {
      auto line = make_vtk<vtkLine>();
      line->GetPointIds()->SetId(0, i);
      line->GetPointIds()->SetId(1, i+4);
      linesArray->InsertNextCell(line);
    }
    
    mpData->SetPoints(points);
    mpData->SetLines(linesArray);
    mpMapper->SetInputData(mpData);
    mpMapper->Modified();
  }
  
  /**
  */
  void VisBounds::setBounds(const IBounds& bounds)
  {
    mMin = bounds.getMin();
    mMax = bounds.getMax();
    update();
  }

}
}