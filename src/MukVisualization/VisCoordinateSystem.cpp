#include "private/muk.pch"
#include "muk_colors.h"

#include "VisCoordinateSystem.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"
#include "MukCommon/MukTransform.h"

#include <vtkSmartPointer.h>

#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkRenderer.h>

namespace gris
{
namespace muk
{

  const int VisCoordinateSystem::FONT_SIZE = 3;
  const double VisCoordinateSystem::MAX_SCALE = 50.0;
  const double VisCoordinateSystem::MIN_SCALE = 30.0;

  /**
  */
  VisCoordinateSystem::VisCoordinateSystem()
    : VisualProp(make_vtk<vtkAxesActor>())
    , mpLabel(make_vtk<vtkCaptionActor2D>())
  {
    auto pAxesActor = getAxesActor();
    pAxesActor->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);
    pAxesActor->AxisLabelsOn();

    mpLabel->ThreeDimensionalLeaderOff();
    mpLabel->BorderOff();
    mpLabel->GetCaptionTextProperty()->SetColor(Colors::White);
    mpLabel->GetCaptionTextProperty()->SetFontSize(FONT_SIZE);

    pAxesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(FONT_SIZE);
    pAxesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(FONT_SIZE);
    pAxesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(FONT_SIZE);

  }

  /**
  */
  VisCoordinateSystem::~VisCoordinateSystem()
  {
    if (mpRenderer)
      mpRenderer->RemoveActor(mpLabel);
  }

  /**
  */
  void VisCoordinateSystem::update()
  {
  }

  /**
  */
  vtkAxesActor* VisCoordinateSystem::getAxesActor()
  {
    // no check against nullptr required, mpActor is guranteed to be a vtkAxesActor
    return vtkAxesActor::SafeDownCast(mpActor);
  }

  /**
  */
  void VisCoordinateSystem::setName(const std::string & name)
  {
    mpLabel->SetCaption(name.c_str());
  }

  void VisCoordinateSystem::setRenderer(vtkRenderer * pRenderer)
  {
    if (mpRenderer) mpRenderer->RemoveActor(mpLabel);
    mpRenderer = pRenderer;
    if (pRenderer) pRenderer->AddActor(mpLabel);
    VisualProp::setRenderer(pRenderer);
  }

  /**
  */
  vtkCaptionActor2D * VisCoordinateSystem::getLabel()
  {
    return mpLabel;
  }

  /**
  */
  void VisCoordinateSystem::setTransform(const vtkSmartPointer<vtkTransform> transform)
  {
    double point[3];
    transform->GetPosition(point);
    getLabel()->SetAttachmentPoint(point);
    VisualProp::setTransform(transform);
    getAxesActor()->SetTotalLength(MAX_SCALE, MAX_SCALE, MAX_SCALE);
  }

  /**
  */
  void VisCoordinateSystem::fade(const double p)
  {
    double fraction = MAX_SCALE; 
    const double div = MIN_SCALE - MAX_SCALE;
    // negative numbers for div are also not faded
    if (div > 1e-6) fraction = p / div + MIN_SCALE;
    getAxesActor()->SetTotalLength(fraction, fraction, fraction);
    mpActor->SetVisibility(true);
    mpLabel->SetVisibility(true);
  }

  void VisCoordinateSystem::setVisibility(const bool state)
  {
    mpActor->SetVisibility(state);
    mpLabel->SetVisibility(state);
  }

  bool VisCoordinateSystem::isVisible() const
  {
    return mpActor->GetVisibility() != 0;
  }

}
}