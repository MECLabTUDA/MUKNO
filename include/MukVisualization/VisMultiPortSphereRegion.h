#pragma once
#include "VisStateRegion.h"

#include <vtkSmartPointer.h>
class vtkImplicitPlaneWidget2;

namespace gris
{
  namespace muk
  {
    class MultiPortSphereRegionRepresentation;

    /** \brief A Visualization Object based on a implicit plane widget with modified representation
    */
    class MUK_VIS_API VisMultiPortSphereRegion : public VisStateRegion
    {
      public:
        VisMultiPortSphereRegion(const std::string& name);
        virtual ~VisMultiPortSphereRegion();

      public:
        virtual void setRenderer(vtkRenderer* pRenderer);

      public:
        virtual const MukState& getCenter() const;
        virtual std::unique_ptr<IStateRegion> asStateRegion(bool asStart) const;
        virtual void setFromProblemDefinition(const MukProblemDefinition& obj);
        virtual bool isMovable() const;

      public:
        vtkImplicitPlaneWidget2* widget();
        MultiPortSphereRegionRepresentation* representation();
        void swap(VisMultiPortSphereRegion* other);

      private:

      private:
        vtkSmartPointer<vtkImplicitPlaneWidget2> mpImpPlaneWidget;
        vtkSmartPointer<MultiPortSphereRegionRepresentation> mpSphereRep;
        unsigned int mResolution;
    };
  }
}
