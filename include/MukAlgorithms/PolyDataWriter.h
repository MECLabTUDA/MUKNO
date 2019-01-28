#pragma once

#include "VtkWrapper.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_ALGO_API PolyDataWriter : public VtkWrapper
    {
      public:
        PolyDataWriter();

      public:
        static  const char* s_name()       { return "PolyDataWriter"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput(unsigned int portId, void* pDataType);
        virtual size_t sizeOutputs() const  { return 0; }
        virtual void   update();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
