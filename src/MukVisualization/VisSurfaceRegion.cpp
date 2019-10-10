#include "private/muk.pch"
#include "VisSurfaceRegion.h"
#include "SurfaceRegionRepresentation.h"
#include "SurfaceRegionWidget.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/SurfaceStateRegion.h"
#include "MukCommon/vtk_tools.h"

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <numeric>

namespace gris
{
namespace muk
{
  /**
  */
  VisSurfaceRegion::VisSurfaceRegion(const std::string& name)
    : VisStateRegion(name)
    , mpWidget(make_vtk<SurfaceRegionWidget>())
  {
    mpRep = dynamic_cast<SurfaceRegionRepresentation*>(mpWidget->GetRepresentation());
    mpWidget->SetProcessEvents(0);
  }

  /**
  */
  VisSurfaceRegion::~VisSurfaceRegion()
  {
  }

  /**
  */
  void VisSurfaceRegion::setVisibility(bool b)
  {
    mpWidget->SetEnabled(b);
  }

  /**
  */
  bool VisSurfaceRegion::getVisibility() const
  {
    return mpWidget->GetEnabled() == 1;
  }

  /**
  */
  void VisSurfaceRegion::setColors(const Vec3d& color)
  {
    mpRep->setArrowColor(color);
  }

  /**
  */
  void VisSurfaceRegion::setRenderer(vtkRenderer* pRenderer)
  {
    VisualObject::setRenderer(pRenderer);
    mpWidget->SetInteractor(pRenderer->GetRenderWindow()->GetInteractor());
    mpWidget->SetEnabled(1); // this will make it visible
    mpWidget->Render();      // this will force a render call so a loaded widget is immediately visible 
  }

  /**
  */
  const MukState& VisSurfaceRegion::getCenter() const
  {
    const auto& points = mpRep->getSurfacePoints();
    auto mass = std::accumulate(points.begin(), points.end(), Vec3d(0.0));
    mass /= points.size();
    mCenter.coords = mass;
    mCenter.tangent = (mpRep->getDirectionAncor()-mass).normalized();
    return mCenter;
  }

  /**
  */
  SurfaceRegionWidget* VisSurfaceRegion::widget()
  {
    return mpWidget.Get();
  }

  /**
  */
  SurfaceRegionRepresentation* VisSurfaceRegion::representation()
  {
    return mpRep;
  }
  /**
  */
  std::unique_ptr<IStateRegion> VisSurfaceRegion::asStateRegion(bool asStart) const
  {
    auto res = std::make_unique<SurfaceStateRegion>();
    res->setDirectionAncor(mpRep->getDirectionAncor());
    res->setPoints(mpRep->getSurfacePoints());
    res->setPointTowardsAncor( ! asStart);
    return std::move(res);
  }

  /**
  */
  bool VisSurfaceRegion::isMovable() const
  {
    return true;
  }

  /**
  */
  void VisSurfaceRegion::swap(VisSurfaceRegion* other)
  {
    std::swap(mpWidget, other->mpWidget);
    std::swap(mpRep, other->mpRep);
  }

  /**
  */
  void VisSurfaceRegion::setSurfacePoints(const std::vector<Vec3d>& p)
  {
    mpRep->setSurfacePoints(p);
  }

  /**
  */
  void VisSurfaceRegion::setDirectionAncor(const Vec3d& p)
  {
    mpRep->setDirectionAncor(p);
  }
}
}