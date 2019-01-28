#include "mukItkIO.h"

#include "MukException.h"

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "itkImage.h"
#include "itkMetaImageIOFactory.h"
#include "itkThresholdImageFilter.h"
#include "itkImageToHistogramFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

namespace gris
{
namespace muk
{
		/** \brief @Returns the mhd file at @filename as an itkImage
	*/	
	itk::Image<unsigned short, 3>::Pointer loadCtData(const std::string& filename)
	{
		boost::filesystem::path path = filename;
		if (0 != path.extension().string().compare(".mhd"))
		{
			throw MUK_EXCEPTION_SIMPLE("Input-file does not have ending 'mhd'!\n");
		}
		typedef unsigned short              PixelType;
		typedef itk::Image<PixelType, 3>    Image;

		auto reader = itk::ImageFileReader<Image>::New();
		itk::MetaImageIOFactory::RegisterOneFactory();
		reader->SetFileName(path.string());
		reader->Update();
			
		itk::Image<unsigned short, 3>::Pointer output = reader->GetOutput();
		return output.GetPointer();
	}
}
}
