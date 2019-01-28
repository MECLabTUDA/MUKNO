#pragma once
#include "VisStateRegion.h"

#include <vtkSmartPointer.h>

namespace gris
{
  namespace muk
  {
    class SphereRegionRepresentation;
    class SphereRegionWidget;

    /** \brief A visualization of an arrow that points in a certain direction together with a partial sphere. The sphere's radius indicates the allowed Euclidean distance. The surface of the partial sphere indicates the allowed angle error. 
    
      Internally a modified implicit plane widget and a modified representation is used to provide a customized visualization and interaction.
    */
    class MUK_VIS_API VisSphereRegion : public VisStateRegion
    {
      public:
        VisSphereRegion(const std::string& name);
        virtual ~VisSphereRegion();

      public:
        virtual void setRenderer(vtkRenderer* pRenderer);
        virtual void setVisibility(bool);
        virtual bool getVisibility()      const;

      public:
        virtual const MukState& getCenter() const;
        virtual std::unique_ptr<IStateRegion> asStateRegion(bool asStart) const;
        virtual void setFromProblemDefinition(const MukProblemDefinition& obj);
        virtual bool isMovable() const;

      public:
        void setState(const MukState& state);

      public:
        SphereRegionWidget*         widget();
        SphereRegionRepresentation* representation();
        void swap(VisSphereRegion* other);
        
      private:
        vtkSmartPointer<SphereRegionWidget> mpWidget;
        SphereRegionRepresentation*         mpRep;
        unsigned int mResolution;
    };
  }
}
