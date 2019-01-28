#include "private/muk.pch"
#include "private/LargestRegionFilter.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkConnectedComponentImageFilter.h>
#include <itkRelabelComponentImageFilter.h>


namespace gris
{
	namespace muk
	{
		void LargestRegionFilter::GenerateInputRequestedRegion()
		{
			Superclass::GenerateInputRequestedRegion();
			for (itk::InputDataObjectIterator it(this); !it.IsAtEnd(); it++)
			{
				auto* input = dynamic_cast<ImageInt3D*>(it.GetInput());
				if (input)
				{
					input->SetRequestedRegionToLargestPossibleRegion();
				}
			}
		}

		/*
		The output is a binary image with the largest connected component remaining
		*/
		void LargestRegionFilter::GenerateData()
		{
			auto* inputImage = this->GetInput(0);

			// Defining output
			const itk::ImageRegion<3U> inputRegion = inputImage->GetLargestPossibleRegion();
			typename ImageInt3D::Pointer output = this->GetOutput();
			output->Initialize();
			output->SetRegions(inputRegion);
			output->Allocate();

			// Using itk::ConnectedComponentImageFilter to separate each connected component
			typedef itk::ConnectedComponentImageFilter <ImageInt3D, ImageInt3D > ConnectedComponentImageFilterType;
			ConnectedComponentImageFilterType::Pointer connected = ConnectedComponentImageFilterType::New();
			connected->SetInput(inputImage);
			connected->Update();

			// Using itk::RelabelComponentImageFilter to relabel each comonents (The largest one has an ID of 1)
			typedef itk::RelabelComponentImageFilter<ImageInt3D, ImageInt3D> RelabelComponentImageFilterType;
			RelabelComponentImageFilterType::Pointer relabelFilter = RelabelComponentImageFilterType::New();
			relabelFilter->SetInput(connected->GetOutput());
			relabelFilter->Update();

			itk::ImageRegionIterator<ImageInt3D> imageIterator(relabelFilter->GetOutput(), inputRegion);
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
