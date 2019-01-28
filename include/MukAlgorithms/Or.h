#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"

#include <itkOrImageFilter.h>

namespace gris
{
  namespace muk
  {
	  /*
	  A morphological filter that performs the logical-or on two images.
	  */
    class MUK_ALGO_API Or : public AlgorithmWrapper
    {
      public:
        Or();

      public:
        static  const char* s_name() { return "Or"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput(unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);
        virtual void   update();

      protected:
        itk::SmartPointer<itk::OrImageFilter<ImageInt3D>>orImageFilter;
        };
  }
}