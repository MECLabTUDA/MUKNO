#include "private/muk.pch"
#include "LeastSquaresFittingLine.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"


namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(LeastSquaresFittingLine);

		/**
		  cast Filter and set properties
		*/
		LeastSquaresFittingLine::LeastSquaresFittingLine()
		{
			mLibType = enItk;
			mDspType = enSegmentation;

			mInputPortTypes.push_back(enImageInt3D);
			mOutputPortTypes.push_back(enImageInt3D);

			leastSquaresFittingLineFilter = make_itk<LeastSquaresFittingLineFilter>();

			declareProperty<EnDisplayType>("DisplayType",
				[=](const auto en) { setDisplayType(en); },
				[=]() { return getDisplayType(); });
		}

		/**
		*/
		void LeastSquaresFittingLine::setInput(unsigned int portId, void* pDataType)
		{
			leastSquaresFittingLineFilter->SetInput(static_cast<ImageInt3D*>(pDataType));
		}

		/**
		*/
		void* LeastSquaresFittingLine::getOutput(unsigned int portId)
		{
			return static_cast<void*>(leastSquaresFittingLineFilter->GetOutput());
		}

		/**
		*/
		void LeastSquaresFittingLine::update()
		{
			leastSquaresFittingLineFilter->UpdateLargestPossibleRegion();
		}
	} // namespace muk
} // namespace gris