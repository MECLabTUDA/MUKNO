#pragma once

#include "private/muk.pch"
#include "MukPath.h"

#include <iterator> // back_inserter

namespace gris
{
namespace muk
{
  /**

    avoid nans for initialization because ifstreams can't properly read them
  */
  MukPath::MukPath()
    : mRadius(0)
    , mGoalAngle(-1)
    , mGoalDist(-1)
    , mNumberOfSearchStates(0)
    , mMillisecondsSpend(0)
    , mApproximated(true)
  {
  }

  /**
  */
  void MukPath::set(MukPath& o)
  {
    this->swap(o);
  }

  /**
  */
  void MukPath::swap(MukPath& o)
  {
    std::swap(mRadius, o.mRadius);
    std::swap(mGoalAngle, o.mGoalAngle);
    std::swap(mGoalDist, o.mGoalDist);
    std::swap(mNumberOfSearchStates, o.mNumberOfSearchStates);
    std::swap(mMillisecondsSpend, o.mMillisecondsSpend);
    mStates.swap(o.getStates());
  }

  /**
  */
  void swap(MukPath& l, MukPath& r)
  {
    l.swap(r);
  }
}
}


namespace std
{
  template<>
  void swap <gris::muk::MukPath>(gris::muk::MukPath& a, gris::muk::MukPath& b)
  {
    a.swap(b);
  }
}
