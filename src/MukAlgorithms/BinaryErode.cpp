#include "private/muk.pch"
#include "BinaryErode.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"



namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(BinaryErode);
	/*
	A morphological filter that performs erode on one image.
	*/
	BinaryErode::BinaryErode()
	{
		mDspType = enDisplayImage3D;
		mInputPortTypes.push_back(enImageInt3D);
		mOutputPortTypes.push_back(enImageInt3D);
		binaryErodeImageFilter = make_itk<itk::BinaryErodeImageFilter<ImageInt3D, ImageInt3D, StructuringElementType>>();
		structuringElement.SetRadius(3);
		structuringElement.CreateStructuringElement();
		binaryErodeImageFilter.GetPointer()->SetKernel(structuringElement);
		binaryErodeImageFilter.GetPointer()->SetErodeValue(1);

		declareProperty<unsigned short>("ErosionValue",
			[=](const unsigned short d) { binaryErodeImageFilter.GetPointer()->SetErodeValue(d); },
			[=]() { return binaryErodeImageFilter.GetPointer()->GetErodeValue(); });
		declareProperty<itk::SizeValueType>("StructuringElementRadius",
			[=](itk::SizeValueType d)
        {
			    structuringElement.SetRadius(d);
			    structuringElement.CreateStructuringElement();
			    binaryErodeImageFilter.GetPointer()->SetKernel(structuringElement);
		    },
			[=]() { return binaryErodeImageFilter.GetPointer()->GetKernel().GetRadius()[0]; });
	}

	void  BinaryErode::setInput(unsigned int portId, void* pDataType)
	{
		binaryErodeImageFilter->SetInput(static_cast<ImageInt3D*>(pDataType));
	}

	/**
	*/
	void* BinaryErode::getOutput(unsigned int portId)
	{
		return static_cast<void*>(binaryErodeImageFilter->GetOutput());
	}

	/**
	*/
	void BinaryErode::update()
	{
		binaryErodeImageFilter->UpdateLargestPossibleRegion();
	}


}
}