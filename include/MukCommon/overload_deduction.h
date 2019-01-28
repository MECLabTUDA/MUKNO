#pragma once

namespace gris
{
  template<typename... Args> struct SELECT { 
    template<typename C, typename R> 
    static constexpr auto OVERLOAD_OF( R (C::*pmf)(Args...) ) -> decltype(pmf) { 
      return pmf;
    } 
  };
}
