#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"
#include "itkAndImageFilter.h"
namespace gris
{
  namespace muk
  {
	  /*
	  A morphological filter that performs the logical-and on two images.
	  */
    class MUK_ALGO_API AndSliceImage : public AlgorithmWrapper
    {
	public:
        AndSliceImage();

	public:
		static  const char* s_name() { return "AndSliceImage"; };
		virtual const char* name()   const { return s_name(); }

	public:
		virtual void   setInput(unsigned int portId, void* pDataType);
		virtual void*  getOutput(unsigned int portId);
		virtual size_t sizeInputs()  const { return 2; }
		virtual size_t sizeOutputs() const { return 1; }
		virtual void   update();

	protected:
		itk::SmartPointer<itk::AndImageFilter<ImageInt2D>>andImageFilter;
    };
  }
}