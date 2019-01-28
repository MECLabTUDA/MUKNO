#pragma once

#include "muk_common_api.h"
#include "MukVector.h"
#include "MukException.h"

#include <string>

namespace gris
{
  namespace muk
  {
        
    MUK_COMMON_API Vec3d stringToVec3d(const std::string& s);
    
    template<class ITER>
    bool hasName(ITER& begin, ITER& end, const std::string& name)
    {
      typedef typename std::iterator_traits<ITER>::value_type Type;
      return end != std::find_if(begin, end, [&] (const Type& s) { return name == s.getName(); });
    }

    template<class ITER>
    inline ITER retrieveName(ITER& begin, ITER& end, const std::string& name)
    {
      return std::find_if(begin, end, [&] (const typename std::iterator_traits<ITER>::value_type& s) { return name == s->getName(); });
    }
    

  }
}
