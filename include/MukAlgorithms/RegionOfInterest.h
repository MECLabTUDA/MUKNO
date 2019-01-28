#pragma once
#include "AlgorithmWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::SobelEdgeDetectionImageFilter
    */
    class MUK_ALGO_API RegionOfInterest : public AlgorithmWrapper
    {
      public:
        RegionOfInterest();

      public:
        static  const char* s_name()       { return "RegionOfInterest"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput (unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);

        virtual void update();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };

  } // namespace muk
} // grsi