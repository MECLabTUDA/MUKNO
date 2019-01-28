#include "private/muk.pch"
#include "ImageFileWriter.h"
#include "AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"
#include <itkImageFileWriter.h>
#include <boost/filesystem.hpp>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(ImageFileWriter);

  // ------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------

  /** \brief 

    cast ImageFileWriter and set properties
  */
  ImageFileWriter::ImageFileWriter()
  {
    mpFilter = make_itk<itk::ImageFileWriter<ImageInt3D>>();
    mDspType = enDisplayImage3D;
    mInputPortTypes.push_back(enImageInt3D);

    auto* p = dynamic_cast<itk::ImageFileWriter<ImageInt3D>*>(mpFilter.GetPointer());
    declareProperty<std::string>("Filename",
      [=] (const auto& str) { p->SetFileName(str); p->Modified(); },
      [=] ()                { return p->GetFileName(); });
  }

  /**
  */
  void ImageFileWriter::setInput(unsigned int portId, void* pDataType)
  {
    using Filter = itk::ImageFileWriter<ImageInt3D>;
    auto pData = toDataType<ImageInt3D>(pDataType);
    static_cast<Filter*>(mpFilter.GetPointer())->SetInput(pData);
  }

  /**
  */
  void ImageFileWriter::update()
  {
    auto* writer = dynamic_cast<itk::ImageFileWriter<ImageInt3D>*>(mpFilter.GetPointer());
    namespace fs = boost::filesystem;
    auto p = fs::path (writer->GetFileName()).parent_path().string();
    if (!p.empty() && !fs::is_directory(p))
    {
      LOG_LINE << "creating directories: " << p;
      fs::create_directories(p);
    }
    mpFilter->UpdateLargestPossibleRegion();
  }
}
}