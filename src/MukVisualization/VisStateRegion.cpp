#include "private/muk.pch"
#include "VisStateRegion.h"

#include "vtkActor.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    VisStateRegion::VisStateRegion(const std::string& name)
      : VisAbstractObject(name)
    {
    }

    /**
    */
    VisStateRegion::~VisStateRegion()
    {
    }
  }
}