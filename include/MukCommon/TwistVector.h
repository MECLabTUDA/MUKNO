#pragma once
#include "LieGroupSE3.h"

namespace gris
{
  namespace se3
  {
    /** \brief A twist in the Lie-Algebra se(3) as 6D vector (E=element)
    */
    class MUK_COMMON_API TwistVector
    {
      public:
        const Vec6d& data() const { return mData; }
        Vec6d&       data()       { return mData; }

      public:
        const double& operator[] (int i) const { return mData[i]; }
        double&       operator[] (int i)       { return mData[i]; }

      public:
        TwistMatrix   hat() const;

      private:
        Vec6d mData;
    };
  }
}