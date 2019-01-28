#include "private/muk.pch"
#include "LargestRegionSlice.h"
#include "AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"


namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(LargestRegionSlice);

		/**
		  cast Filter and set properties
		*/
		LargestRegionSlice::LargestRegionSlice()
		{
			mLibType = enItk;
			mDspType = enSegmentation;
			mInputPortTypes.push_back(enImageInt2D);
			mOutputPortTypes.push_back(enImageInt2D);
			largestRegionFilter = make_itk<LargestRegionSliceFilter>();
		}

		/**
		*/
		void  LargestRegionSlice::setInput(unsigned int portId, void* pDataType)
		{
			largestRegionFilter->SetInput(static_cast<ImageInt2D*>(pDataType));
		}

		/**
		*/
		void* LargestRegionSlice::getOutput(unsigned int portId)
		{
			return static_cast<void*>(largestRegionFilter->GetOutput());
		}

		/**
		*/
		void LargestRegionSlice::update()
		{
			largestRegionFilter->UpdateLargestPossibleRegion();
		}
	} // namespace muk
} // namespace gris