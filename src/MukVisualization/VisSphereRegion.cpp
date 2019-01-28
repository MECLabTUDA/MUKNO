#include "private/muk.pch"
#include "VisSphereRegion.h"
#include "SphereRegionRepresentation.h"
#include "SphereRegionWidget.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukProblemDefinition.h"
#include "MukCommon/MukStateRegion.h"

#include "MukCommon/gris_math.h"

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>


namespace gris
{
  namespace muk
  {
    /**
    */
    VisSphereRegion::VisSphereRegion(const std::string& name)
      : VisStateRegion(name)
      , mpWidget(make_vtk<SphereRegionWidget>())
      , mResolution(1)
    {
      mpRep = dynamic_cast<SphereRegionRepresentation*>(mpWidget->GetRepresentation());
      mpWidget->SetProcessEvents(0);

      declareProperty<unsigned int>("SampleResolution",
        [&] (unsigned int p)  { mResolution = p; },
        [&] ()                { return mResolution; });
      declareProperty<double>("Phi", 
        [&] (double d) { mpRep->setSpherePhi(d); }, 
        [&] ()         { return mpRep->getSpherePhi(); });
      declareProperty<double>("Scale", 
        [&] (double d) { mpRep->SetScale(d); }, 
        [&] ()         { return mpRep->GetScale(); });
    }

    /**
    */
    VisSphereRegion::~VisSphereRegion()
    {
    }

    /**
    */
    void VisSphereRegion::setVisibility(bool b)
    {
      mpWidget->SetEnabled(b);
    }

    /**
    */
    bool VisSphereRegion::getVisibility() const
    {
      return mpWidget->GetEnabled() == 1;
    }

    /**
    */
    void VisSphereRegion::setRenderer(vtkRenderer* pRenderer)
    {
      VisualObject::setRenderer(pRenderer);
      mpWidget->SetInteractor(pRenderer->GetRenderWindow()->GetInteractor());
      mpWidget->SetEnabled(1); // this will make it visible
    }

    /**
    */
    const MukState& VisSphereRegion::getCenter() const
    {
      mCenter = mpRep->getState();
      return mCenter;
    }

    /**
    */
    SphereRegionWidget* VisSphereRegion::widget()
    {
      return mpWidget.Get(); 
    }
    
    /**
    */
    SphereRegionRepresentation* VisSphereRegion::representation()
    {
      return mpRep; 
    }

    /** \brief Creates a #MukStateRegion from the visualization

      \param asStart: indicates wether it should be build as a start or goal region. In the latter case the direction is swapped.
    */
    std::unique_ptr<IStateRegion> VisSphereRegion::asStateRegion(bool asStart) const
    {
      auto res = std::make_unique<MukStateRegion>();
      auto state = mpRep->getState();
      if ( ! asStart)
        state.tangent *= -1;
      res->setCenter(state);
      const auto phi = mpRep->getSpherePhi() * gris::M_PI / 180.0;;
      res->setPhi(phi);
      res->setRadius(mpRep->getSphereRadius());
      res->setResolution(mResolution);
      return std::move(res);
    }

    /**
    */
    bool VisSphereRegion::isMovable() const
    {
      return true;
    }

    /** \brief
    */
    void VisSphereRegion::swap(VisSphereRegion* other)
    {
      std::swap(mpWidget, other->mpWidget);
      std::swap(mpRep, other->mpRep);
      std::swap(mResolution, other->mResolution);
    }

    /**
    */
    void VisSphereRegion::setFromProblemDefinition(const MukProblemDefinition& obj)
    {
      mpRep->setSpherePhi ( obj.getGoalAngleThreshold() * 180 / M_PI );
    }

    /**
    */
    void VisSphereRegion::setState(const MukState& state)
    {
      mpRep->setSphereOrigin(state.coords);
      mpRep->setArrowDirection(state.tangent);
    }
  }
}