#include "private/muk.pch"
#include "private/FreeSpaceConfinement.h"
#include "private/tools.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkImageRegionConstIterator.h>
#include <itkPasteImageFilter.h>
#include <itkOrImageFilter.h>
#include <itkNotImageFilter.h>

namespace gris
{
namespace muk
{
  /** \brief needs the entire image, i.e. LargestPossibleRegion

  check itk superclass for more details
  */
  void FreeSpaceConfinement::GenerateInputRequestedRegion()
  {
    Superclass::GenerateInputRequestedRegion();
    for (itk::InputDataObjectIterator it(this); ! it.IsAtEnd(); ++it)
    {
      auto* input = dynamic_cast<ImageInt3D*>( it.GetInput() );
      if (input)
      {
        input->SetRequestedRegionToLargestPossibleRegion();
      }
    }
  }

  /**
  */
  void FreeSpaceConfinement::GenerateData()
  {
    // prepare input and output
    auto inputInnerSoftTissue = make_itk<ImageInt3D>();
    inputInnerSoftTissue->Graft(const_cast<ImageInt3D*>(this->GetInput()));
    auto inputOuterSoftTissue = make_itk<ImageInt3D>();
    inputOuterSoftTissue->Graft(const_cast<ImageInt3D*>(this->GetInput(1)));
    auto inputImgAir = make_itk<ImageInt3D>();
    inputImgAir->Graft(const_cast<ImageInt3D*>(this->GetInput(2)));
    auto inputImgAirConfined = make_itk<ImageInt3D>();
    inputImgAirConfined->Graft(const_cast<ImageInt3D*>(this->GetInput(3)));

    this->AllocateOutputs();
    auto* output = this->GetOutput(0);
    allocateFromSrc(inputOuterSoftTissue, output);

    // compute temporal bone tissue
    auto or = make_itk<itk::OrImageFilter<ImageInt3D>>();
    or->SetInput1(inputInnerSoftTissue);
    or->SetInput2(inputOuterSoftTissue);
    auto not = make_itk<itk::NotImageFilter<ImageInt3D,ImageInt3D>>();
    not->SetInput(or->GetOutput());
    not->Update();
    
    const auto size = inputOuterSoftTissue->GetLargestPossibleRegion();

    ImageInt3D::IndexType minAirIdx = size.GetUpperIndex();
    ImageInt3D::IndexType maxAirIdx = size.GetIndex();
    {
      auto it = itk::ImageRegionConstIterator<ImageInt3D>(inputImgAir, inputImgAir->GetBufferedRegion());
      for (it.GoToBegin(); !it.IsAtEnd(); ++it)
      {
        if (it.Get())
        {
          auto idx = it.GetIndex();
          minimize(idx, minAirIdx);
          maximize(idx, maxAirIdx);
        }
      }
    }
    ImageInt3D::IndexType minConfinedIdx = size.GetUpperIndex();
    ImageInt3D::IndexType maxConfinedIdx = size.GetIndex();
    {
      auto it = itk::ImageRegionConstIterator<ImageInt3D>(inputImgAirConfined, inputImgAirConfined->GetBufferedRegion());
      for (it.GoToBegin(); !it.IsAtEnd(); ++it)
      {
        if (it.Get())
        {
          auto idx = it.GetIndex();
          minimize(idx, minConfinedIdx);
          maximize(idx, maxConfinedIdx);
        }
      }
    }

    ImageInt3D::IndexType boundsMin = size.GetUpperIndex();
    ImageInt3D::IndexType boundsMax = size.GetIndex();
    {
      auto it = itk::ImageRegionConstIterator<ImageInt3D>(or->GetOutput(), or->GetOutput()->GetBufferedRegion());
      for (it.GoToBegin(); !it.IsAtEnd(); ++it)
      {
        if (it.Get())
        {
          auto idx = it.GetIndex();
          minimize(idx, boundsMin);
          maximize(idx, boundsMax);
        }
      }
    }

    ImageInt3D::IndexType minIdx, maxIdx;
    // adjust regions
    // medial of the air is not interesting, posterior slight more than the air boundary (10 pixels should be enough to include the bone
    minIdx[0] = minAirIdx[0];
    maxIdx[0] = maxAirIdx[0];
    // anterior of the confined air regions is not interesting, posterior unntil the mid of the confined air region (roughly EAC)
    minIdx[1] = 0.5*(maxConfinedIdx[1] + minConfinedIdx[1]);
    maxIdx[1] = maxAirIdx[1];
    // slightly superior to the air cavities and inferior until end of the air region
    minIdx[2] = std::max((ImageInt3D::IndexValueType)0, minAirIdx[2]);
    maxIdx[2] = maxConfinedIdx[2] + 0.5*(maxConfinedIdx[2]-minConfinedIdx[2]);
    // make sure, the index is not out of bounds
    for (size_t i(0); i<3; ++i)
    {
      minIdx[i] = std::max(boundsMin[i], minIdx[i]);
      maxIdx[i] = std::min(boundsMax[i], maxIdx[i]);
    }

    itk::ImageRegion<3u> roi;
    roi.SetIndex(minIdx);
    roi.SetUpperIndex(maxIdx);
    auto paste = make_itk<itk::PasteImageFilter<ImageInt3D, ImageInt3D>>();
    paste->SetSourceImage(not->GetOutput());
    paste->SetDestinationImage(output);
    paste->SetSourceRegion(roi);
    paste->SetDestinationIndex(roi.GetIndex());
    paste->Update();

    output->Graft(paste->GetOutput());
  }
}
}