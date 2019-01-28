#include "private/muk.pch"
#include "And.h"
#include "AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(And);

	/* \brief A morphological filter that performs the logical-and on two images.
	*/
	And::And()
	{
		mDspType = enDisplayOverlay3D;

		mInputPortTypes.push_back(enImageInt3D);
		mInputPortTypes.push_back(enImageInt3D);
		mOutputPortTypes.push_back(enImageInt3D);
		andImageFilter = make_itk<itk::AndImageFilter<ImageInt3D>>();
	}

	void  And::setInput(unsigned int portId, void* pDataType)
	{
		switch (portId)
		{
		case 0:
			andImageFilter->SetInput(0, static_cast<ImageInt3D*>(pDataType));
			break;
		case 1:
			andImageFilter->SetInput(1, static_cast<ImageInt3D*>(pDataType));
			break;
		}
	}

	/**
	*/
	void* And::getOutput(unsigned int portId)
	{
		return static_cast<void*>(andImageFilter->GetOutput());
	}

	/**
	*/
	void And::update()
	{
		andImageFilter->UpdateLargestPossibleRegion();
	}


}
}