#include "private/muk.pch"
#include "private/LargestRegionSliceFilter.h"
#include "MukImaging/muk_imaging_tools.h"
#include <itkConnectedComponentImageFilter.h>
#include <itkRelabelComponentImageFilter.h>


namespace gris
{
	namespace muk
	{
		void LargestRegionSliceFilter::GenerateInputRequestedRegion()
		{
			Superclass::GenerateInputRequestedRegion();
			for (itk::InputDataObjectIterator it(this); !it.IsAtEnd(); it++)
			{
				auto* input = dynamic_cast<ImageInt2D*>(it.GetInput());
				if (input)
				{
					input->SetRequestedRegionToLargestPossibleRegion();
				}
			}
		}

		/*
		The output is a binary image with the largest connected component remaining
		*/
		void LargestRegionSliceFilter::GenerateData()
		{
			auto* inputImage = this->GetInput(0);

			// Defining output
			const itk::ImageRegion<2U> inputRegion = inputImage->GetLargestPossibleRegion();
			typename ImageInt2D::Pointer output = this->GetOutput();
			output->Initialize();
			output->SetRegions(inputRegion);
			output->Allocate();

			// Using itk::ConnectedComponentImageFilter to separate each connected component
			typedef itk::ConnectedComponentImageFilter <ImageInt2D, ImageInt2D > ConnectedComponentImageFilterType;
			ConnectedComponentImageFilterType::Pointer connected = ConnectedComponentImageFilterType::New();
			connected->SetInput(inputImage);
			connected->Update();

			// Using itk::RelabelComponentImageFilter to relabel each comonents (The largest one has an ID of 1)
			typedef itk::RelabelComponentImageFilter<ImageInt2D, ImageInt2D> RelabelComponentImageFilterType;
			RelabelComponentImageFilterType::Pointer relabelFilter = RelabelComponentImageFilterType::New();
			relabelFilter->SetInput(connected->GetOutput());
			relabelFilter->Update();

			itk::ImageRegionIterator<ImageInt2D> imageIterator(relabelFilter->GetOutput(), inputRegion);
			while (!imageIterator.IsAtEnd()) {
				if (imageIterator.Get() == 1) 
				{
					output->SetPixel(imageIterator.GetIndex(), 1);
				}
				else
				{
					output->SetPixel(imageIterator.GetIndex(), 0);
				}
				++imageIterator;
			}
		}
	}
}
