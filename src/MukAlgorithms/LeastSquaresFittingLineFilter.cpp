#include "private/muk.pch"
#include "private/LeastSquaresFittingLineFilter.h"
#include "MukImaging/muk_imaging_tools.h"


#include <CGAL/linear_least_squares_fitting_2.h>
#include <CGAL/Simple_cartesian.h>
#include <itkLineIterator.h>
#include <itkImageRegionIterator.h>
#include <vtkSmartPointer.h>
#include <vtkLineSource.h>
#include <vtkPolyData.h>
namespace gris
{
	namespace muk
	{
		void LeastSquaresFittingLineFilter::GenerateInputRequestedRegion()
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
		
		*/
		void LeastSquaresFittingLineFilter::GenerateData()
		{
			// input is a 3d mask
			auto* inputImage = this->GetInput(0);

			// image specification
			ImageInt3D::RegionType region;
			region = inputImage->GetRequestedRegion();
			std::cout << "Region: " << region << std::endl;


			// create 2d mask
			typedef itk::Image< unsigned char, 2 >  MaskType;
			MaskType::RegionType maskregion;
			MaskType::IndexType maskstart;
			maskstart[0] = 0;
			maskstart[1] = 0;
			MaskType::SizeType masksize;
			masksize[0] = region.GetSize()[0];
			masksize[1] = region.GetSize()[1];
			maskregion.SetSize(masksize);
			maskregion.SetIndex(maskstart);
			MaskType::Pointer mask = MaskType::New();
			mask->SetRegions(maskregion);
			mask->Allocate();
			mask->FillBuffer(0);

			// convert 3d mask to 2d mask
			itk::ImageRegionConstIterator<ImageInt3D> imageIterator(inputImage, region);
			imageIterator.GoToBegin();
			while (!imageIterator.IsAtEnd()) {
				if (imageIterator.Get()) {
					MaskType::IndexType index;
					index[0] = imageIterator.GetIndex()[0];
					index[1] = imageIterator.GetIndex()[1];
					mask->SetPixel(index, 1);
				}
				++imageIterator;
			}


			// PCA
			typedef double FT;
			typedef CGAL::Simple_cartesian<FT>  K;
			typedef K::Line_2                   Line;
			typedef K::Point_2                  Point;
			std::vector<Point> points;

			// adding coordinates to points
			itk::ImageRegionIterator<MaskType> maskIterator(mask, maskregion);
			maskIterator.GoToBegin();
			while (!maskIterator.IsAtEnd()) {
				if (maskIterator.Get()) {
					// current pixel is 1
					points.push_back(Point(maskIterator.GetIndex()[0], maskIterator.GetIndex()[1]));
				}
				++maskIterator;
			}

			// calculate the line
			Line line;
			linear_least_squares_fitting_2(points.begin(), points.end(), line, CGAL::Dimension_tag<0>());

			// coefficients of the line
			double a = line.a();
			double b = line.b();
			double c = line.c();
			// ax + by + c = 0
			// y = (-ax - c) /b


			// with this we can calculate two points on the line that lies within the image
			double y_min = (-a * 0 - c) / b;
			double y_max = (-a * (masksize[0] - 1) - c) / b;
			// in 2d
			/*
			MaskType::IndexType p1;
			p1[0] = 0;
			p1[1] = y_min;
			MaskType::IndexType p2;
			p2[0] = masksize[0] - 1;
			p2[1] = y_max;
			*/

			double p1[3] = { 0.0,  y_min, 0.0 };
			double p2[3] = { masksize[0] - 1, y_max, 0.0 };
			vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
			lineSource->SetPoint1(p1);
			lineSource->SetPoint2(p2);
			lineSource->Update();


			// Defining output
			const itk::ImageRegion<3U> inputRegion = inputImage->GetLargestPossibleRegion();
			typename ImageInt3D::Pointer output = this->GetOutput();
			output->Initialize();
			output->SetRegions(inputRegion);
			output->Allocate();
			output->FillBuffer(0);





		}
	}
}
