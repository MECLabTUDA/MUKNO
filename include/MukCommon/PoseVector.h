#pragma once
#include "LieGroupSE3.h"

namespace gris
{
  namespace se3
  {
    /** \brief A pose (position and orientation) of the special Euclidean group SE(3) as a 6D vector
    */
    class MUK_COMMON_API PoseVector
    {
      public:
        const Vec6d& data() const { return mData; }

      public:
        const double& operator[] (int i) const { return mData[i]; }
        double&       operator[] (int i)       { return mData[i]; }

      private:
        Vec6d mData;
    };
  }
}