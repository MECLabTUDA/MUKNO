#pragma once

#include "muk_common_api.h"

#include <atomic>

namespace gris
{
  namespace muk
  {
    class MUK_COMMON_API TimerOwner
    {
    public:
      TimerOwner();
      // copies do not share Timers
      TimerOwner(const TimerOwner& rhs);

    public:
      virtual const char* name() const { return "TimerOwner"; }

    public:
      const std::string& getIdentifier() const;

    private:
      static std::atomic<unsigned int> COUNTER;

      std::string mIdentifier;
    };

  }
}

