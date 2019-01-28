#pragma once
#include "ITKWrapper.h"

#include <itkMorphologicalWatershedFromMarkersImageFilter.h>

namespace gris
{
  namespace muk
  {
    /** \brief 
    */
    class MUK_ALGO_API WatershedSliceFilter : public AlgorithmWrapper
    {
      public:
          WatershedSliceFilter();

      public:
        static  const char* s_name()       { return "WatershedSliceFilter"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput(unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);
        virtual void   update();

      protected:
        itk::SmartPointer<itk::MorphologicalWatershedFromMarkersImageFilter<ImageInt2D, ImageInt2D>>morphWatershedFilter;
    };

  } // namespace muk
} // grsi