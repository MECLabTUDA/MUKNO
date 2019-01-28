#include "private/muk.pch"
#include "private/LayerGrowing.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkImageRegionConstIterator.h>
#include <itkImageRegionIterator.h>

namespace gris
{
namespace muk
{
  /** \brief needs the entire image, i.e. LargestPossibleRegion

  check itk superclass for more details
  */
  void LayerGrowing::GenerateInputRequestedRegion()
  {
    Superclass::GenerateInputRequestedRegion();
    for( itk::InputDataObjectIterator it( this ); !it.IsAtEnd(); it++ )
    {
      auto* input = dynamic_cast<ImageInt3D*>( it.GetInput() );
      if ( input )
      {
        input->SetRequestedRegionToLargestPossibleRegion();
      }
    }
  }

  /**
  */
  void LayerGrowing::GenerateData()
  {
    // prepare input and output
    auto input = make_itk<ImageInt3D>();
    input->Graft( const_cast<ImageInt3D*>( this->GetInput() ));
    auto* labelImage = this->GetInput(1);
    this->AllocateOutputs();
    auto* output = this->GetOutput(0);
    {
      output->SetRegions(input->GetRequestedRegion());
      output->SetOrigin(input->GetOrigin());
      output->SetDirection(input->GetDirection());
      output->SetSpacing(input->GetSpacing());
      output->Allocate();
    }
    // fill output with src
    {
      auto src = itk::ImageRegionConstIterator<ImageInt3D>(input, input->GetBufferedRegion());
      auto tar = itk::ImageRegionIterator<ImageInt3D>(output, output->GetBufferedRegion());
      while (!src.IsAtEnd())
      {
        tar.Set(src.Get());
        ++src;
        ++tar;
      }
    }
    using Pixel = ImageInt3D::PixelType;
    // determine maximum label
    Pixel maxValue(0);
    auto maxIter = itk::ImageRegionConstIterator<ImageInt3D>(labelImage, labelImage->GetBufferedRegion());
    while (!maxIter.IsAtEnd())
    {
      maxValue = std::max(maxValue, maxIter.Get());
      ++maxIter;
    }
    // run through the labels
    const auto size = input->GetLargestPossibleRegion().GetSize();
    for (Pixel i(1); i<=maxValue; ++i)
    {
      auto iter    = itk::ImageRegionConstIterator<ImageInt3D>(labelImage, labelImage->GetBufferedRegion());
      auto outIter = itk::ImageRegionIterator<ImageInt3D>(output, output->GetBufferedRegion());
      while (!iter.IsAtEnd())
      {
        auto val = iter.Get();
        if (val==i)
        {
          auto idx = iter.GetIndex();
          // check 6-neighborhood
          // direction x
          for (int j(0); j<3; ++j)
          {
            if (idx[j]>0)
            {
              auto nextIdx = idx;
              nextIdx[j] -= 1;
              auto label = output->GetPixel(nextIdx);
              if (label)
              {
                outIter.Set(label);
                break;
              }
            }
            if ((int)idx[j]<(int)size[j]-1)
            {
              auto nextIdx = idx;
              nextIdx[j] += 1;
              auto label = output->GetPixel(nextIdx);
              if (label)
              {
                outIter.Set(label);
                break;
              }
            }
          }
        }
        ++iter;
        ++outIter;
      }
    }
  }
}
}