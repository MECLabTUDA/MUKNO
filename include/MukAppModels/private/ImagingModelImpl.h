#pragma once
#include "ImagingModel.h"

#include "MukCommon/muk_common.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

namespace gris
{
  namespace muk
  {
    /** \brief private implementation of WorldVisualizationModel
    */
    struct ImagingModel::Impl
    {
      Impl();

      MukImage::Pointer mpImage;  // the CT volume
    };

    /**
    */
    ImagingModel::Impl::Impl()
    {
      mpImage = make_itk<MukImage>();
    }
  }
}
