#include "private/muk.pch"
#include "SliceImageReader.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkImageFileReader.h>

namespace gris
{
namespace muk
{
	REGISTER_ALGORITHM(SliceImageReader);

	/** 
    Data source that reads image data from a single file.
	*/
	SliceImageReader::SliceImageReader()
	{
		mpFilter = make_itk<itk::ImageFileReader<ImageInt2D>>();
		mDspType = enDisplayImage2D;

    mOutputPortTypes.push_back(enImageInt2D);

		auto* p = dynamic_cast<itk::ImageFileReader<ImageInt2D>*>(mpFilter.GetPointer());
		declareProperty<std::string>("Filename",
			[=](const auto& str) { p->SetFileName(str); p->Modified(); },
			[=]() { return p->GetFileName(); });
	}
}
}