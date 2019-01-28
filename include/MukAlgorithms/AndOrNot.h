#pragma once
#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Creates a binary/Segmentation image by thresholding at a lower and/or upper limit.
    */
    class MUK_ALGO_API AndOrNot : public AlgorithmWrapper
    {
      public:
        AndOrNot();

      public:
        static  const char* s_name()       { return "AndOrNot"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void  setInput (unsigned int portId, void* pDataType);
        virtual void* getOutput(unsigned int portId) { return static_cast<void*>(mpFilter->GetOutput()); }
        virtual void  update() { mpFilter->UpdateLargestPossibleRegion(); }

      protected:
        struct Impl;
        std::unique_ptr<Impl> mp;
        itk::SmartPointer<itk::ImageToImageFilter<ImageInt3D, ImageInt3D>> mpFilter;
    };

  } // namespace muk
} // gris