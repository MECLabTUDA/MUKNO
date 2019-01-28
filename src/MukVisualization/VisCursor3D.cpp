#include "private/muk.pch"
#include "private/ArrowMaker.h"
#include "VisCursor3D.h"

#include "PolyDataHandler.h"
#include "muk_colors.h"

#include <vtkActor.h>
#include <vtkArrowSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

namespace gris
{
namespace muk
{
  /**
  */
  VisCursor3D::VisCursor3D(const std::string& name)
    : VisAbstractObject(name)
    , mpArrowSource(make_vtk<vtkArrowSource>())
    , mpArrowTransform(make_vtk<vtkTransform>())
    , mpArrowFilter(make_vtk<vtkTransformPolyDataFilter>())
    , mShowArrow(false)
  {
    mState.tangent = Vec3d(1,0,0);
    auto points = make_vtk<vtkPoints>();
    points->InsertNextPoint(0,0,0);
    mpData->SetPoints(points);
    PolyDataHandler::addVertices(mpData);
    mpActor->GetProperty()->SetPointSize(5.0);
    computeArrow();
    mDefaultColor = Vec3d(Colors::Green);
    setColors(mDefaultColor);
    
    declareProperty<bool>("ShowArrow"
      , [&] (bool b) { this->setArrowVisible(b); }
      , [&] () { return this->arrowIsVisible(); });
  }

  /**
  */
  VisCursor3D::~VisCursor3D()
  {
  }

  /**
  */
  void VisCursor3D::setPosition(const Vec3d& p)
  {
    mState.coords = p;
    computeArrow();
  }

  /**
  */
  void VisCursor3D::setDirection(const Vec3d& p)
  {
    mState.tangent = p;
    computeArrow();
  }

  /**
  */
  void VisCursor3D::setState(const MukState& p)
  {
    mState = p;
    computeArrow();
  }

  /**
  */
  Vec3d VisCursor3D::getPosition() const
  {
    return mState.coords;
  }

  Vec3d VisCursor3D::getDirection() const
  {
    return mState.tangent;
  }

  /**
  */
  void VisCursor3D::computeArrow()
  {
    if (mShowArrow)
    {
      mpArrowFilter->SetTransform(mpArrowTransform);
      mpArrowFilter->SetInputConnection(mpArrowSource->GetOutputPort());
      mpMapper->SetInputConnection(mpArrowFilter->GetOutputPort());
      createArrow(mState, mpArrowTransform);
    }
    else
    {
      mpData->GetPoints()->SetPoint(0, mState.coords.x(), mState.coords.y(), mState.coords.z());
      mpData->Modified();
      mpMapper->SetInputData(mpData);
    }
    mpMapper->Modified();
  }

  /**
  */
  bool VisCursor3D::arrowIsVisible() const
  {
    return mShowArrow;
  }

  /**
  */
  void VisCursor3D::setArrowVisible(bool b)
  {
    mShowArrow = b;
    computeArrow();
  }
}
}