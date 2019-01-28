#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"

#include <itkBinaryBallStructuringElement.h>
#include <itkBinaryErodeImageFilter.h>

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::BinaryErodeImageFilter
    */
    class MUK_ALGO_API BinaryErode : public AlgorithmWrapper
    {
	    public:
		    BinaryErode();

	    public:
		    static  const char* s_name() { return "BinaryErode"; };
		    virtual const char* name()   const { return s_name(); }

	    public:
		    virtual void   setInput(unsigned int portId, void* pDataType);
		    virtual void*  getOutput(unsigned int portId);
		    virtual void   update();

	    protected:
		    using  StructuringElementType = itk::BinaryBallStructuringElement<ImageInt3D::PixelType, ImageInt3D::ImageDimension>;
		    StructuringElementType structuringElement;

		    itk::SmartPointer<itk::BinaryErodeImageFilter<ImageInt3D, ImageInt3D, StructuringElementType>> binaryErodeImageFilter;
    };
  }
}