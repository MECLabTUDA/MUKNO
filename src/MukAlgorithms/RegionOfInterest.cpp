#include "private/muk.pch"
#include "RegionOfInterest.h"
#include "AlgorithmFactory.h"
#include "muk_algorithms_tools.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkPasteImageFilter.h>


namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(RegionOfInterest);

    struct RegionOfInterest::Impl
    {
      ImageInt3D*         input;
      ImageInt3D::Pointer output;

      void setModified()    { mModified = true; }
      bool modified() const { return mModified; }

      Vec3i min;
      Vec3i max;

      bool mModified = true;
    };

    /** 
    */
    RegionOfInterest::RegionOfInterest()
      : mp (std::make_unique<Impl>())
    {
      mDspType = enDisplayImage3D;
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);

      mp->output= make_itk<ImageInt3D>();

      declareProperty<Vec3i>("LowerBound",
        [=] (const auto& val) { mp->min= val; mp->setModified(); },
        [=] ()                { return mp->min; });
      declareProperty<Vec3i>("UpperBound",
        [=] (const auto& val) { mp->max= val; mp->setModified(); },
        [=] ()                { return mp->max; });
    }

    /**
    */
    void RegionOfInterest::setInput(unsigned int portId, void* pDataType)
    {
      switch (portId)
      {
        case 0:
          mp->input = toDataType<ImageInt3D>(pDataType);
          break;
        case 1:
          mp->output = toDataType<ImageInt3D>(pDataType);
          break;
      };
      mp->setModified();
    }

    /**
    */
    void* RegionOfInterest::getOutput(unsigned int portId)
    {
      return static_cast<void*>(mp->output.GetPointer());
    }

    /** \brief check if an update is needed, then create the pipeline and copy the result
    */
    void RegionOfInterest::update()
    {
      if ( ! mp->modified())
        return;

      auto filter = make_itk<itk::PasteImageFilter<ImageInt3D, ImageInt3D>>();
      itk::ImageRegion<3u> region;
      ImageInt3D::IndexType targetIndex, minIndex, maxIndex;
      for (int i(0); i<3; ++i)
      {
        minIndex[i] = mp->min[i];
        maxIndex[i] = mp->max[i];
        targetIndex[i] = mp->min[i];
      }
      region.SetIndex(minIndex);
      region.SetUpperIndex(maxIndex);

      auto black = make_itk<ImageInt3D>();
      allocateFromSrc(mp->input, black);

      filter->SetSourceImage(mp->input);
      filter->SetSourceRegion(region);
      filter->SetDestinationImage(black);
      filter->SetDestinationIndex(targetIndex);
      
      filter->Update();
      mp->output->Graft(filter->GetOutput());

      mp->mModified = false;
    }
  } // namespace muk
} // namespace gris