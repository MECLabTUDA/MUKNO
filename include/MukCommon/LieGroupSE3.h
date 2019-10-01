#pragma once
#include "muk_common_api.h"
#include "Quaternion.h"
#include "StaticMatrix.h"

#include <gstd/Vector.h>

namespace gris
{
  namespace se3
  {
    /** \brief a six dimensional vector
    */
    using Vec6d = gris::Vector<double, double, 6>;
    class PoseMatrix;
    class PoseVector;
    class TwistMatrix;
    class TwistVector;
  }
}

#include "PoseMatrix.h"
#include "PoseVector.h"
#include "TwistMatrix.h"
#include "TwistVector.h"