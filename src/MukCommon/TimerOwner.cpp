#pragma once

#include "private/muk.pch"
#include "TimerOwner.h"

namespace gris
{
  namespace muk
  {
    std::atomic<unsigned int> TimerOwner::COUNTER = 0;

    TimerOwner::TimerOwner()
    {
      mIdentifier = name() + ("_" + std::to_string(++COUNTER));
    }

    TimerOwner::TimerOwner(const TimerOwner & rhs) : TimerOwner()
    {
    }

    const std::string & TimerOwner::getIdentifier() const
    {
      return mIdentifier;
    }

  }
}