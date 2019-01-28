#include "private/muk.pch"
#include "WatershedSliceFilter.h"
#include "AlgorithmFactory.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(WatershedSliceFilter);

  /**
  */
  WatershedSliceFilter::WatershedSliceFilter()
  {
    mDspType = enDisplayOverlay2D;
    mInputPortTypes.push_back(enImageInt2D);
    mInputPortTypes.push_back(enImageInt2D);
    mOutputPortTypes.push_back(enImageInt2D);
    morphWatershedFilter = make_itk<itk::MorphologicalWatershedFromMarkersImageFilter<ImageInt2D, ImageInt2D>>();
  }

  void  WatershedSliceFilter::setInput(unsigned int portId, void* pDataType)
  {
    switch (portId)
    {
    case 0:
        morphWatershedFilter->SetInput1(static_cast<ImageInt2D*>(pDataType));
        break;
    case 1:
        morphWatershedFilter->SetInput2(static_cast<ImageInt2D*>(pDataType));
        break;
    }
  }

  /**
  */
  void* WatershedSliceFilter::getOutput(unsigned int portId)
  {
      return static_cast<void*>(morphWatershedFilter->GetOutput());
  }

  /**
  */
  void WatershedSliceFilter::update()
  {
      morphWatershedFilter->UpdateLargestPossibleRegion();
  }
}
}