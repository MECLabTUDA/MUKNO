#pragma once

#include "muk_common_api.h"
#include "muk_common.h"

#include "MukVector.h"

#include <vector>
#include <ostream>

Forward_Declare_Muk_Boost_Serialization

namespace gris
{
namespace muk
{
  /** \brief An elemnet of SE2 := R^3 x S^2. Consists of position and direction.
  */
  class MUK_COMMON_API MukState
  {
    public:
      MukState();
      MukState(const Vec3d& c, const Vec3d& t);
      MukState(const MukState& o);
      MukState& operator=(const MukState& o);
      MukState(MukState&&);
      MukState& operator=(MukState&& o);
      ~MukState();
    
    public:
      bool operator== (const MukState& rhs) const;
      bool operator!= (const MukState& rhs) const;

    private:
      friend MUK_COMMON_API std::ostream& operator<<(std::ostream& os, const MukState& state);

    public:
      void swap(MukState& o);

    public:
      Vec3d coords;
      Vec3d tangent;

      Use_Muk_Boost_Serialization
  };
  
  MUK_COMMON_API void swap(MukState& a, MukState& b);  
}
}

namespace std
{
  template<>
  MUK_COMMON_API void swap <gris::muk::MukState> (gris::muk::MukState& a, gris::muk::MukState& b);
}
