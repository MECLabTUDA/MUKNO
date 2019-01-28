#include "private/muk.pch"
#include "SliceImageFileWriter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkImageFileWriter.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(SliceImageFileWriter);

  // ------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------

  /** \brief 

    cast SliceImageFileWriter and set properties
  */
  SliceImageFileWriter::SliceImageFileWriter()
  {
    mpFilter = make_itk<itk::ImageFileWriter<ImageInt2D>>();
    mDspType = enDisplayImage2D;

    auto* p = dynamic_cast<itk::ImageFileWriter<ImageInt2D>*>(mpFilter.GetPointer());
    declareProperty<std::string>("Filename", [=] (const auto& str) { p->SetFileName(str); }, [=] () { return p->GetFileName(); });
  }

  /**
  */
  void SliceImageFileWriter::setInput(unsigned int portId, void* pDataType)
  {
    using Filter = itk::ImageFileWriter<ImageInt2D>;
    auto pData = toDataType<ImageInt2D>(pDataType);
    static_cast<Filter*>(mpFilter.GetPointer())->SetInput(pData);
  }
}
}