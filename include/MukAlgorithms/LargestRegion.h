#pragma once
#include "ITKWrapper.h"

#include "private\LargestRegionFilter.h"

namespace gris
{
  namespace muk
  {
    /*
    The output is a binary image with the largest connected component remaining
    */
    class MUK_ALGO_API LargestRegion : public AlgorithmWrapper
    {
      public:
        LargestRegion();

      public:
        static  const char* s_name() { return "LargestRegion"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput(unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);
        virtual void   update();

      protected:
        itk::SmartPointer<LargestRegionFilter> largestRegionFilter;
    };
  }
} // gris