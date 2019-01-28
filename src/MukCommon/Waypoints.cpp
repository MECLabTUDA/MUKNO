#pragma once

#include "private/muk.pch"
#include "Waypoints.h"

#include "MukException.h"

#include <boost/format.hpp>

#include <exception>
#include <utility> // swap
#include <iterator>

namespace gris
{
  namespace muk
  {

    MukState Waypoints::InitStartState = MukState(Vec3d(0,0,0), Vec3d(0,0,0));
    MukState Waypoints::InitGoalState  = MukState(Vec3d(0,0,0), Vec3d(0,1,1));

    /**
    */
    Waypoints::Waypoints()
    {
      mStates.resize(2);
      mStates[0] = InitStartState;
      mStates[1] = InitGoalState;
    }

    /**
    */
    Waypoints::Waypoints(const Waypoints& o)
    {
      std::copy(o.states().begin(), o.states().end(), std::back_inserter(mStates));
    }

    /**
    */
    Waypoints& Waypoints::operator=(const Waypoints& o)
    {
      if (this != &o)  // self-assignment check expected
      { 
        mStates.resize(o.states().size());
        std::copy(o.states().begin(), o.states().end(), mStates.begin());
      }
      return *this;
    }

    /**
    */
    Waypoints::~Waypoints()
    {
    }

    /**
    */
    void Waypoints::resize(size_t n)
    {
      if (n<2)
      {
        throw MUK_EXCEPTION("requested size is below minimum of 2 states", (boost::format("requested size: %d") % n).str().c_str());
      }
      mStates.resize(n);
    }

    /**
    */
    void Waypoints::removePoint(size_t index)
    {
      if (index == 0 || index >= mStates.size())
      {
        throw MUK_EXCEPTION_SIMPLE( (boost::format("Waypoints: invalid index %d! (size %d)!") % index % mStates.size()).str().c_str() );
      }
      mStates.erase(mStates.begin()+index);
    }

    void Waypoints::swap(Waypoints& o)
    {
      using std::swap;
      swap(mStates, o.mStates);
    }

    void swap(Waypoints& a, Waypoints& b)
    {
      a.swap(b);
    }

    /*std::ostream& operator<<(std::ostream& os, const Waypoints& data)
    {
      return data.print(os);
    }

    std::ostream& Waypoints::print(std::ostream& os)
    {
      for (const auto& state : mStates)
        print(os),
    }

*/
  }
}


