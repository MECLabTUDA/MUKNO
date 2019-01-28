#include "private/muk.pch"
#include "Or.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(Or);

	/*
	A morphological filter that performs the logical-or on two images.
	*/
	Or::Or()
	{
		mDspType = enDisplayOverlay3D;

		mInputPortTypes.push_back(enImageInt3D);
		mInputPortTypes.push_back(enImageInt3D);
		mOutputPortTypes.push_back(enImageInt3D);
		orImageFilter = make_itk<itk::OrImageFilter<ImageInt3D>>();
	}

	void  Or::setInput(unsigned int portId, void* pDataType)
	{
		switch (portId)
		{
		case 0:
			orImageFilter->SetInput(0, static_cast<ImageInt3D*>(pDataType));
			break;
		case 1:
			orImageFilter->SetInput(1, static_cast<ImageInt3D*>(pDataType));
			break;
		}
	}

	/**
	*/
	void* Or::getOutput(unsigned int portId)
	{
		return static_cast<void*>(orImageFilter->GetOutput());
	}

	/**
	*/
	void Or::update()
	{
		orImageFilter->UpdateLargestPossibleRegion();
	}
}
}