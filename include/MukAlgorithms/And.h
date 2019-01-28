#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"
#include "itkAndImageFilter.h"
namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::AndImageFilter
    */
    class MUK_ALGO_API And : public AlgorithmWrapper
    {
	public:
		And();

	public:
		static  const char* s_name() { return "And"; };
		virtual const char* name()   const { return s_name(); }

	public:
		virtual void   setInput(unsigned int portId, void* pDataType);
		virtual void*  getOutput(unsigned int portId);
		virtual void   update();

	protected:
		itk::SmartPointer<itk::AndImageFilter<ImageInt3D>>andImageFilter;
    };
  }
}