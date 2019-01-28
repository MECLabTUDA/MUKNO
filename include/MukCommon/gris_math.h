#pragma once

#ifdef M_PI
#undef M_PI
#endif

namespace gris
{
  // from <math.h>
  constexpr double M_E        = 2.71828182845904523536;   // e
  constexpr double M_LOG2E    = 1.44269504088896340736;   // log2(e)
  constexpr double M_LOG10E   = 0.434294481903251827651;  // log10(e)
  constexpr double M_LN2      = 0.693147180559945309417;  // ln(2)
  constexpr double M_LN10     = 2.30258509299404568402;   // ln(10)
  constexpr double M_PI       = 3.14159265358979323846;   // pi
  constexpr double M_PI_2     = 1.57079632679489661923;   // pi/2
  constexpr double M_PI_4     = 0.785398163397448309616;  // pi/4
  constexpr double M_1_PI     = 0.318309886183790671538;  // 1/pi
  constexpr double M_2_PI     = 0.636619772367581343076;  // 2/pi
  constexpr double M_2_SQRTPI = 1.12837916709551257390;   // 2/sqrt(pi)
  constexpr double M_SQRT2    = 1.41421356237309504880;   // sqrt(2)
  constexpr double M_SQRT1_2  = 0.707106781186547524401;  // 1/sqrt(2)
}

