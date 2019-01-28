#include "private/muk.pch"
#include "VisPlaneRegion.h"
#include "PlaneRegionRepresentation.h"
#include "PlaneRegionWidget.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"
#include "MukCommon/PlaneStateRegion.h"
#include "MukCommon/MukProblemDefinition.h"

#include <vtkImageData.h>
#include <vtkImplicitPlaneWidget2.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>


namespace gris
{
  namespace muk
  {
    /**
    */
    VisPlaneRegion::VisPlaneRegion(const std::string& name)
      : VisStateRegion(name)
      , mpWidget(make_vtk<PlaneRegionWidget>())
      , mResolution(1.0)
    {
      mpRep = reinterpret_cast<PlaneRegionRepresentation*>(mpWidget->GetRepresentation());
      mpWidget->SetProcessEvents(0);

      declareProperty<double>("SampleResolution",
        [&] (double p)  { mResolution = p; },
        [&] ()          { return mResolution; });
    }

    /**
    */
    VisPlaneRegion::~VisPlaneRegion()
    {
    }

    /**
    */
    PlaneRegionWidget* VisPlaneRegion::widget()
    {
      return mpWidget.Get();
    }

    /**
    */
    PlaneRegionRepresentation* VisPlaneRegion::representation()
    { 
      return mpRep; 
    }

    /**
    */
    void VisPlaneRegion::setVisibility(bool b)
    {
      mpWidget->SetEnabled(b);
    }

    /**
    */
    bool VisPlaneRegion::getVisibility() const
    {
      return mpWidget->GetEnabled() == 1;
    }

    /**
    */
    void VisPlaneRegion::setRenderer(vtkRenderer* pRenderer)
    {
      VisualObject::setRenderer(pRenderer);
      mpWidget->SetInteractor(pRenderer->GetRenderWindow()->GetInteractor());
      mpWidget->SetEnabled(1); // this will make it visible
      mpWidget->Render();      // this will force a render call so a loaded widget is immediately visible 
    }

    /**
    */
    const MukState& VisPlaneRegion::getCenter() const
    {
      mCenter = MukState(Vec3d(mpRep->GetOrigin()), Vec3d(1,0,0));
      return mCenter;
    }

    /**
    */
    std::unique_ptr<IStateRegion> VisPlaneRegion::asStateRegion(bool asStart) const
    {
      auto res = std::make_unique<PlaneStateRegion>();
      res->setOrigin(Vec3d(mpRep->GetOrigin()));
      res->setNormal(Vec3d(mpRep->GetNormal()));
      res->setDirectionAncor(mpRep->getDirectionAncor());
      res->setPointsTowardsAncor( ! asStart);
      res->setBoundingPolygon(mpRep->getBoundingPolygon());
      res->setResolution(mResolution);
      return std::move(res);
    }

    /**
    */
    bool VisPlaneRegion::isMovable() const
    {
      return true;
    }

    /** \brief
    */
    void VisPlaneRegion::swap(VisPlaneRegion* other)
    {
      std::swap(mpWidget, other->mpWidget);
      std::swap(mpRep, other->mpRep);
      std::swap(mResolution,other->mResolution);
    }

    /**
    */
    void VisPlaneRegion::setPlane(const Vec3d& origin, const Vec3d& normal)
    {
      mpRep->setPlane(origin, normal);
    }

    /**
    */
    void VisPlaneRegion::setDirectionAncor(const Vec3d& p)
    {
      mpRep->setDirectionAncor(p);
    }
    
    /**
    */
    void VisPlaneRegion::setBounds(const std::vector<Vec3d>& points)
    {
      mpRep->setBounds(points);
    }
  }
}