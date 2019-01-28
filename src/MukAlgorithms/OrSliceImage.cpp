#include "private/muk.pch"
#include "OrSliceImage.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"



namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(OrSliceImage);

	/*
	  A morphological filter that performs the logical-or on two images.
	*/
	OrSliceImage::OrSliceImage()
	{
		mDspType = enDisplayOverlay2D;

		mInputPortTypes.push_back(enImageInt2D);
		mInputPortTypes.push_back(enImageInt2D);
		mOutputPortTypes.push_back(enImageInt2D);
		orImageFilter = make_itk<itk::OrImageFilter<ImageInt2D>>();
	}

	void  OrSliceImage::setInput(unsigned int portId, void* pDataType)
	{
		switch (portId)
		{
		case 0:
			orImageFilter->SetInput(0, static_cast<ImageInt2D*>(pDataType));
			break;
		case 1:
			orImageFilter->SetInput(1, static_cast<ImageInt2D*>(pDataType));
			break;
		}
	}

	/**
	*/
	void* OrSliceImage::getOutput(unsigned int portId)
	{
		return static_cast<void*>(orImageFilter->GetOutput());
	}

	/**
	*/
	void OrSliceImage::update()
	{
		orImageFilter->UpdateLargestPossibleRegion();
	}
}
}