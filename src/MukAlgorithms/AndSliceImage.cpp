#include "private/muk.pch"
#include "AndSliceImage.h"
#include "AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"



namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(AndSliceImage);
	/*
	A morphological filter that performs the logical-and on two images.
	*/
	AndSliceImage::AndSliceImage()
	{
		mLibType = enItk;
		mDspType = enSegmentation;
		mInputPortTypes.push_back(enImageInt2D);
		mInputPortTypes.push_back(enImageInt2D);
		mOutputPortTypes.push_back(enImageInt2D);
		andImageFilter = make_itk<itk::AndImageFilter<ImageInt2D>>();
	}

	void  AndSliceImage::setInput(unsigned int portId, void* pDataType)
	{
		switch (portId)
		{
		case 0:
			andImageFilter->SetInput(0, static_cast<ImageInt2D*>(pDataType));
			break;
		case 1:
			andImageFilter->SetInput(1, static_cast<ImageInt2D*>(pDataType));
			break;
		}
	}

	/**
	*/
	void* AndSliceImage::getOutput(unsigned int portId)
	{
		return static_cast<void*>(andImageFilter->GetOutput());
	}

	/**
	*/
	void AndSliceImage::update()
	{
		andImageFilter->UpdateLargestPossibleRegion();
	}


}
}