#pragma once

#include <chrono>

namespace gris
{
namespace muk
{
  class VisTimeDelays
  {
  public:
    typedef std::chrono::milliseconds Time;

  public:
    VisTimeDelays(const Time timeToFade = Time::zero(), const Time timeToStopFade = Time::zero(), const Time timeToHide = Time::zero());
    VisTimeDelays(const VisTimeDelays& other);
    ~VisTimeDelays();

    bool any() const;

    template <class duration>
    duration getTimeToFade() const
    {
      return mTimeToFade;
    }

    template <class duration>
    duration setTimeToFade(const duration& time) const
    {
      mTimeToFade = time;
    }

    template <class duration>
    duration getTimeToStopFade() const
    {
      return mTimeToStopFade;
    }

    template <class duration>
    duration setTimeToStopFade(const duration& time) const
    {
      mTimeToStopFade = time;
    }

    template <class duration>
    duration getTimeToHide() const
    {
      return mTimeToHide;
    }

    template <class duration>
    duration setTimeToHide(const duration& time) const
    {
      mTimeToHide = time;
    }
  private:
    Time mTimeToFade;
    Time mTimeToHide;
    Time mTimeToStopFade;
  };


}
}