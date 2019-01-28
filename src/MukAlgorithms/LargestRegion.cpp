#include "private/muk.pch"
#include "LargestRegion.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"


namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(LargestRegion);

		/**
		  cast Filter and set properties
		*/
		LargestRegion::LargestRegion()
		{
			mDspType = enDisplayOverlay3D;

			mInputPortTypes.push_back(enImageInt3D);
			mOutputPortTypes.push_back(enImageInt3D);

			largestRegionFilter = make_itk<LargestRegionFilter>();
		}

		/**
		*/
		void  LargestRegion::setInput(unsigned int portId, void* pDataType)
		{
			largestRegionFilter->SetInput(static_cast<ImageInt3D*>(pDataType));
		}

		/**
		*/
		void* LargestRegion::getOutput(unsigned int portId)
		{
			return static_cast<void*>(largestRegionFilter->GetOutput());
		}

		/**
		*/
		void LargestRegion::update()
		{
			largestRegionFilter->UpdateLargestPossibleRegion();
		}
	} // namespace muk
} // namespace gris