#pragma once

#include "AlgorithmWrapper.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_ALGO_API OpenClose3D : public AlgorithmWrapper
    {
      public:
        OpenClose3D();

      public:
        static  const char* s_name()       { return "OpenClose3D"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput(unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);
        virtual void   update();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
