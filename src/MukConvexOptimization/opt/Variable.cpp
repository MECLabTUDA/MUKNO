#include "opt/Variable.h"

#include <iostream>

namespace gris
{
namespace opt
{
  /**
  */
  std::ostream& Variable::operator<<(std::ostream& os) const
  {
    if (mpRep)
      os << mpRep->getName();
    else
      os << "nullvar";
    return os;
  }
}
}