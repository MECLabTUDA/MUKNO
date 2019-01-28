#pragma once
#include "ITKWrapper.h"

#include "MukCommon/muk_common.h"
#include  <itkBinaryBallStructuringElement.h>
#include <itkBinaryDilateImageFilter.h>
namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::BinaryDilateImageFilter
    */
    class MUK_ALGO_API BinaryDilate : public AlgorithmWrapper
    {
	    public:
		    BinaryDilate();

	    public:
		    static  const char* s_name() { return "BinaryDilate"; };
		    virtual const char* name()   const { return s_name(); }

	    public:
		    virtual void   setInput(unsigned int portId, void* pDataType);
		    virtual void*  getOutput(unsigned int portId);
		    virtual void   update();

	    protected:
		    using  StructuringElementType = itk::BinaryBallStructuringElement<ImageInt3D::PixelType, ImageInt3D::ImageDimension>;
		    StructuringElementType structuringElement;

		    itk::SmartPointer<itk::BinaryDilateImageFilter<ImageInt3D, ImageInt3D, StructuringElementType>> binaryDilateImageFilter;
    };
  }
}