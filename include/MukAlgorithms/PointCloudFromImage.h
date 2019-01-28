#pragma once
#include "AlgorithmWrapper.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_ALGO_API PointCloudFromImage : public AlgorithmWrapper
    {
      public:
        PointCloudFromImage();

      public:
        static  const char* s_name()       { return "PointCloudFromImage"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput (unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);
        virtual size_t sizeInputs()  const { return 1; }
        virtual size_t sizeOutputs() const { return 1; }
        virtual void   update();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
