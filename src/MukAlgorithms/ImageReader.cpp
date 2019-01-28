#include "private/muk.pch"
#include "ImageReader.h"
#include "AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"
#include <itkImageFileReader.h>

namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(ImageReader);

	/** 
    Data source that reads image data from a single file.
	*/
	ImageReader::ImageReader()
	{
		mpFilter = make_itk<itk::ImageFileReader<ImageInt3D>>();
		mDspType = enDisplayImage3D;
    mOutputPortTypes.push_back(enImageInt3D);

		auto* p = dynamic_cast<itk::ImageFileReader<ImageInt3D>*>(mpFilter.GetPointer());
    p->SetFileName("M:/MUKNO2/Rohdaten/data/preoperative/general/P01/P01_CT.mhd");
		declareProperty<std::string>("Filename",
			[=](const auto& str) { p->SetFileName(str); mpFilter->Modified(); },
			[=]() { return p->GetFileName(); });
	}
}
}