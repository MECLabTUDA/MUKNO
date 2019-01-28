#include "private/muk.pch"
#include "VisTimeDelays.h"

#include <algorithm>
#include <array>

namespace gris
{
namespace muk
{
  /**
  */
  VisTimeDelays::VisTimeDelays(
    const Time timeToFade, 
    const Time timeToStopFade, 
    const Time timeToHide)
    : mTimeToFade(timeToFade)
    , mTimeToHide(timeToHide)
    , mTimeToStopFade(timeToStopFade)
  {  }

  /**
  */
  VisTimeDelays::VisTimeDelays(const VisTimeDelays & other)
    : mTimeToFade(other.mTimeToFade)
    , mTimeToHide(other.mTimeToHide)
    , mTimeToStopFade(other.mTimeToStopFade)
  {  }

  /**
  */
  VisTimeDelays::~VisTimeDelays()
  {
  }

  bool VisTimeDelays::any() const
  {
    const std::array<const Time*, 3> a{ &mTimeToFade, &mTimeToHide, &mTimeToStopFade };
    return std::any_of(a.begin(), a.end(), [a](const Time* p) {return *p != Time::zero(); });
  }

}
}