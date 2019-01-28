#include "private/muk.pch"
#include "BinaryNot.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"


namespace gris
{
	namespace muk
	{
		REGISTER_ALGORITHM(BinaryNot);

		/**
		*/
		BinaryNot::BinaryNot() 
    {
			using Type = itk::BinaryNotImageFilter<ImageInt3D>;
			mpFilter = make_itk<Type>();
			mDspType = enDisplayOverlay3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

			auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());
			ptr->SetForegroundValue(1);
		}
	}
}