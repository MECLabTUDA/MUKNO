#pragma once
#include "VisAbstractObject.h"

#include "MukCommon/MukState.h"
#include "MukCommon/IStateRegion.h"

namespace gris
{
  namespace muk
  {
    class MukProblemDefinition;

    /**
    */
    class MUK_VIS_API VisStateRegion : public VisAbstractObject
    {
      public:
        explicit VisStateRegion(const std::string& name);
        virtual ~VisStateRegion();

      public:
        virtual const MukState& getCenter() const = 0;
        virtual std::unique_ptr<IStateRegion> asStateRegion(bool asStart) const = 0;
        virtual bool isMovable() const = 0;
        virtual void setFromProblemDefinition(const MukProblemDefinition& obj) = 0;
      
      protected:
        mutable MukState mCenter;
    };

  }
}
