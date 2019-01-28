#include "private/muk.pch"
#include "BinaryDilate.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"



namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(BinaryDilate);
	/*
	A morphological filter that performs dilation on one image.
	*/
	BinaryDilate::BinaryDilate()
	{
		mDspType = enDisplayOverlay3D;

		mInputPortTypes.push_back(enImageInt3D);
		mOutputPortTypes.push_back(enImageInt3D);

		binaryDilateImageFilter = make_itk<itk::BinaryDilateImageFilter<ImageInt3D, ImageInt3D, StructuringElementType>>();

		structuringElement.SetRadius(3);
		structuringElement.CreateStructuringElement();
		binaryDilateImageFilter.GetPointer()->SetKernel(structuringElement);
		binaryDilateImageFilter.GetPointer()->SetDilateValue(1);

		declareProperty<unsigned short>("DilationValue",
			[=] (unsigned short d)  { binaryDilateImageFilter.GetPointer()->SetDilateValue(d); },
			[=] ()                  { return binaryDilateImageFilter.GetPointer()->GetDilateValue(); });

		declareProperty<itk::SizeValueType>("StructuringElementRadius",
			[=] (itk::SizeValueType d) 
      {
			  structuringElement.SetRadius(d);
			  structuringElement.CreateStructuringElement();
			  binaryDilateImageFilter.GetPointer()->SetKernel(structuringElement);
		  },
			[=]() { return binaryDilateImageFilter.GetPointer()->GetKernel().GetRadius()[0]; });
	}

	void  BinaryDilate::setInput(unsigned int portId, void* pDataType)
	{
		binaryDilateImageFilter->SetInput(static_cast<ImageInt3D*>(pDataType));
	}

	/**
	*/
	void* BinaryDilate::getOutput(unsigned int portId)
	{
		return static_cast<void*>(binaryDilateImageFilter->GetOutput());
	}

	/**
	*/
	void BinaryDilate::update()
	{
		binaryDilateImageFilter->UpdateLargestPossibleRegion();
	}


}
}