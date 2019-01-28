#pragma once

#include "AlgorithmWrapper.h"

namespace gris
{
	namespace muk
	{
		/** \brief Class to wrapp itk::SobelEdgeDetectionImageFilter
		*/
		class MUK_ALGO_API SobelEdgeDetectionImageFilter : public AlgorithmWrapper
		{
		public:
			SobelEdgeDetectionImageFilter();

		public:
			virtual void  setInput(unsigned int portId, void* pDataType)
			{
				auto pData = toDataType<ImageInt3D>(pDataType);
				inputFilter->SetInput(pData);
				outputFilter->SetInput(inputFilter->GetOutput());
			}

			virtual void* getOutput(unsigned int portId)
			{
				return static_cast<void*>(outputFilter->GetOutput());
			}

			virtual size_t sizeInputs() const
			{
				return 1;
			}

			virtual size_t sizeOutputs() const
			{
				return 1;
			}

			virtual void update()
			{
				outputFilter->Update();
			}

			static  const char* s_name() { return "SobelEdgeDetectionImageFilter"; };
			virtual const char* name()   const { return s_name(); }

		protected:
			itk::SmartPointer<itk::ImageToImageFilter<ImageInt3D, ImageFloat3D>> inputFilter;
			itk::SmartPointer<itk::ImageToImageFilter<ImageFloat3D, ImageInt3D>> outputFilter;
		};

	} // namespace muk
} // gris

