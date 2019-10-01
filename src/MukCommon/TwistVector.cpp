#include "private/muk.pch"
#include "TwistVector.h"

namespace gris
{
namespace se3
{
  /** \brief The 'hat' or 'wedge' operator ^: R^6 -> R^(4x4)

  see Murray et al: Appendix A, eq (A.2), Example A.8
  \input el: an element of se(3) represented by a vector in R^6
  */
  TwistMatrix TwistVector::hat() const
  {
    TwistMatrix twist;
    // incremental translation
    const auto& xi = mData;
    for (int i(0); i<3; ++i)
      twist(i, 3) = xi[i];
    // incremental rotation
    const auto w1 = xi[3];
    const auto w2 = xi[4];
    const auto w3 = xi[5];
    twist(0, 1) = -w3;
    twist(0, 2) = w2;
    twist(1, 0) = w3;
    twist(1, 2) = -w1;
    twist(2, 0) = -w2;
    twist(2, 1) = w1;
    return twist;
  }
}
}