#include "private/muk.pch"
#include "private/SoftTissueClassification.h"

#include "private/tools.h"
#include "private/LayerGrowing.h"

#include "MukCommon/MukException.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkImageRegionConstIterator.h>
#include <itkImageRegionReverseConstIterator.h>
#include <itkImageSliceConstIteratorWithIndex.h>

#include <itkBinaryThresholdImageFilter.h>
#include <itkConnectedComponentImageFilter.h>
#include <itkConnectedThresholdImageFilter.h>
#include <itkDanielssonDistanceMapImageFilter.h>
#include <itkNotImageFilter.h>
#include <itkOrImageFilter.h>
#include <itkAndImageFilter.h>
#include <itkRegionOfInterestImageFilter.h>
#include <itkPasteImageFilter.h>
#include <itkExtractImageFilter.h>
#include <itkImageLinearConstIteratorWithIndex.h>

#include <itkImageFileWriter.h>

namespace
{
  using namespace gris::muk;
}

namespace gris
{
namespace muk
{
  /**
  */
  SoftTissueClassification::SoftTissueClassification()
  {
    this->SetNumberOfRequiredOutputs(4);
    this->SetNumberOfRequiredInputs(0);

    this->SetNthOutput( 0, this->MakeOutput(0) );
    this->SetNthOutput( 1, this->MakeOutput(1) );
    this->SetNthOutput( 2, this->MakeOutput(2) );
    this->SetNthOutput( 3, this->MakeOutput(3) );
  }

  /** \brief needs the entire image, i.e. LargestPossibleRegion

    check itk superclass for more details
  */
  void SoftTissueClassification::GenerateInputRequestedRegion()
  {
    Superclass::GenerateInputRequestedRegion();
    for( itk::InputDataObjectIterator it( this ); !it.IsAtEnd(); ++it )
    {
      auto* input = dynamic_cast<MukImage*>( it.GetInput() );
      if ( input )
      {
        input->SetRequestedRegionToLargestPossibleRegion();
      }
    }
  }

  /** \brief computes the actual output image
  */
  void SoftTissueClassification::GenerateData()
  {
    auto ctImage = make_itk<MukImage>();
    ctImage->Graft( static_cast<MukImage*>( this->GetInputs()[0].GetPointer() ));
    auto segImage = make_itk<MukImage>();
    segImage->Graft( static_cast<MukImage*>( this->GetInputs()[1].GetPointer() ));
    auto eacImage = make_itk<MukImage>();
    eacImage->Graft( static_cast<MukImage*>( this->GetInputs()[2].GetPointer() ));

    const auto imgSize = ctImage->GetLargestPossibleRegion().GetSize();
    // determine a bounding box around the temporal bone
    auto subregion = estimateSubRegion(segImage);
    // compute a first approximation of soft tissue that does not include voxels within the temporal bone
    auto imgOuterSoftTissue = computeOuterSoftTissue(subregion);
    // this img contains all kind of soft tissue now. 
    // the next step differentiates between soft tissue outside of the skull and inside of it
    auto imgLateralSoftTissue = computeLateralSoftTissue(subregion, imgOuterSoftTissue);
    // compute soft tissue around the lower jaw
    auto imgLowerJawSoftTissue = computeLowerJawSoftTissue(subregion, imgOuterSoftTissue);
    // allocate output
    this->AllocateOutputs();
    for (int i(0); i<4; ++i)
      allocateFromSrc(ctImage, this->GetOutput(i));
    // we fill the second output with a partial solution and copy a temporal solution of the first output into tmpImg
    auto tmpImg = make_itk<MukImage>();
    allocateFromSrc(ctImage, tmpImg);
    // fill outputs
    {
      auto iter1 = itk::ImageRegionConstIterator<MukImage>(imgOuterSoftTissue, subregion);
      auto iter2 = itk::ImageRegionConstIterator<MukImage>(imgLateralSoftTissue, subregion);
      auto iter3 = itk::ImageRegionConstIterator<MukImage>(imgLowerJawSoftTissue, subregion);
      auto target1 = itk::ImageRegionIterator<MukImage>(tmpImg, subregion);
      auto target2 = itk::ImageRegionIterator<MukImage>(this->GetOutput(1), subregion);
      while ( ! iter1.IsAtEnd())
      {
        if ( iter2.Get() || iter3.Get() )
        {
          target2.Set(1);
        }
        else if ( iter1.Get() )
        {
          target1.Set(1);
        }
        ++iter1;
        ++iter2;
        ++iter3;
        ++target1;
        ++target2;
      }
    }
    // now tmpImg stil has a huge volume at the bottom. 
    // the bottom soft tissue is connected to the brain via the jugular vein (which can be very large) and the carotid artery
    LOG_LINE << "substep 7";
    Regions medialRegions, lateralRegions;
    auto partialResult = make_itk<MukImage>();
    allocateFromSrc(ctImage, partialResult);
    extractBrain(subregion, tmpImg, eacImage, medialRegions, lateralRegions, partialResult);
    
    // write results
    auto* output1 = this->GetOutput(0);
    for (const auto region : medialRegions)
      for (const auto& idx : region)
    {
      output1->SetPixel(idx, 1);
    }
    auto* output2 = this->GetOutput(1);
    for (const auto region : lateralRegions)
      for (const auto& idx : region)
    {
      output2->SetPixel(idx, 1);
    }
    LOG_LINE << "substep 8";
    auto* output3 = this->GetOutput(2);
    auto out3Src = itk::ImageRegionConstIterator<MukImage>(imgLateralSoftTissue, imgLateralSoftTissue->GetBufferedRegion());
    auto out3    = itk::ImageRegionIterator<MukImage>(output3, output3->GetBufferedRegion());
    while (!out3.IsAtEnd())
    {
      out3.Set(out3Src.Get());
      ++out3;
      ++out3Src;
    }
    auto* output4 = this->GetOutput(3);
    auto out4Src = itk::ImageRegionConstIterator<MukImage>(imgOuterSoftTissue, imgOuterSoftTissue->GetBufferedRegion());
    auto out4 = itk::ImageRegionIterator<MukImage>(output4, output4->GetBufferedRegion());
    while (!out4.IsAtEnd())
    {
      out4.Set(out4Src.Get());
      ++out4;
      ++out4Src;
    }
  }

  /** \brief computes a region of interest based on the segmentation of the air cavities
  */
  itk::ImageRegion<3u> SoftTissueClassification::estimateSubRegion(MukImage* pSegmentationImage)
  {
    MukImage::IndexType maxIndex, minIndex;
    for (int i(0); i<3; ++i)
    {
      maxIndex[i] = 0;
      minIndex[i] = pSegmentationImage->GetBufferedRegion().GetSize()[i];
    }
    auto iter = itk::ImageRegionConstIterator<MukImage>(pSegmentationImage, pSegmentationImage->GetBufferedRegion());
    while (!iter.IsAtEnd())
    {
      if (iter.Get())
      {
        auto idx = iter.GetIndex();
        minimize(idx, minIndex);
        maximize(idx, maxIndex);
      }
      ++iter;
    }
    itk::ImageRegion<3u> subregion;
    subregion.SetIndex(minIndex);
    subregion.SetUpperIndex(maxIndex);
    return subregion;
  }

  /** \brief 
  */
  MukImage::Pointer SoftTissueClassification::computeOuterSoftTissue(const ROI& subregion)
  {
    auto ctImage = make_itk<MukImage>();
    ctImage->Graft( static_cast<MukImage*>( this->GetInputs()[0].GetPointer() ));
    auto segImage = make_itk<MukImage>();
    segImage->Graft( static_cast<MukImage*>( this->GetInputs()[1].GetPointer() ));

    auto img = make_itk<MukImage>();
    {
      img->SetRegions  (ctImage->GetRequestedRegion());
      img->SetOrigin   (ctImage->GetOrigin());
      img->SetDirection(ctImage->GetDirection());
      img->SetSpacing  (ctImage->GetSpacing());
      img->Allocate();
      img->FillBuffer(0);

      auto thresh = make_itk<itk::BinaryThresholdImageFilter<MukImage, MukImage>> ();
      auto distance = make_itk<itk::DanielssonDistanceMapImageFilter<MukImage, MukImage>>();
      distance->SetInput(segImage);
      thresh->SetInput(distance->GetOutput());;
      thresh->SetLowerThreshold(mLowerThresh);
      thresh->GetOutput()->SetRequestedRegion(subregion);
      thresh->Update();

      auto distanceBrain = make_itk<itk::DanielssonDistanceMapImageFilter<MukImage, MukImage>>();
      distanceBrain->SetInput(thresh->GetOutput());
      auto threshBrain = make_itk<itk::BinaryThresholdImageFilter<MukImage, MukImage>> ();
      threshBrain->SetInput(distanceBrain->GetOutput());
      threshBrain->SetUpperThreshold(mLowerThresh);
      threshBrain->GetOutput()->SetRequestedRegion(subregion);
      threshBrain->Update();

      auto threshIter      = itk::ImageRegionConstIterator<MukImage>(thresh->GetOutput(), subregion);
      auto threshBrainIter = itk::ImageRegionConstIterator<MukImage>(threshBrain->GetOutput(), subregion);
      auto segIter         = itk::ImageRegionConstIterator<MukImage>(segImage, subregion);

      auto target = itk::ImageRegionIterator<MukImage>(img, subregion);
      while ( ! threshIter.IsAtEnd())
      {
        bool goodHit = threshIter.Get() || threshBrainIter.Get();
        bool badHit  = segIter.Get();
        if (goodHit && ! badHit)
          target.Set(1);
        ++threshIter;
        ++threshBrainIter;
        ++segIter;
        ++target;
      }
    }
    return img;
  }

  /**
  */
  MukImage::Pointer SoftTissueClassification::computeLateralSoftTissue(const ROI& subregion, MukImage* imgOuterSoftTissue)
  {
    auto ctImage = make_itk<MukImage>();
    ctImage->Graft( static_cast<MukImage*>( this->GetInputs()[0].GetPointer() ));
    auto rgLateralSoftTissue = make_itk<itk::ConnectedThresholdImageFilter<MukImage, MukImage>>();
    // region growing works on the whole image => need to create a new one with only the subregion labeled
    auto img = make_itk<MukImage>();
    allocateFromSrc(ctImage, img);
    // a seed based region growing from somewhere at the lateral side and until half the depth of the temporal bone should give a good first separation
    itk::ImageRegion<3u> nextRegion;
    auto lower = subregion.GetIndex();
    auto upper = subregion.GetUpperIndex();
    lower[0] += static_cast<MukImage::IndexType::IndexValueType>( 0.5  * (upper[0] - lower[0]) );
    nextRegion.SetIndex(lower);
    nextRegion.SetUpperIndex(upper);
    auto seedIter = itk::ImageSliceConstIteratorWithIndex<MukImage>( imgOuterSoftTissue, nextRegion);
    seedIter.SetFirstDirection(0);
    seedIter.SetSecondDirection(1);
    bool iterate = true;
    seedIter.GoToBegin();
    while (!seedIter.IsAtEnd() && iterate)    {
      while (!seedIter.IsAtEndOfSlice() && iterate)      {
        while (!seedIter.IsAtEndOfLine() && iterate)
        {
          if (seedIter.Get())
          {
            iterate = false;
          }
          ++seedIter;
        }
        seedIter.NextLine();
      }
      seedIter.NextSlice();
    }
   /* for (seedIter.GoToBegin(); ! seedIter.IsAtEnd() && iterate; seedIter.NextSlice())
      for (seedIter.GoToBeginOfSlice(); ! seedIter.IsAtEndOfSlice() && iterate; seedIter.NextLine())
        for (seedIter.Be ; !seedIter.IsAtEndOfLine() && iterate; ++seedIter)
        {
          if (seedIter.Get())
          {
            iterate = false;
          }
        }*/
    if (iterate)
    {
      throw MUK_EXCEPTION_SIMPLE("Warning: Seed Iterator out of bounds");
    }
    auto src = itk::ImageRegionConstIterator<MukImage>( imgOuterSoftTissue, nextRegion);
    auto target = itk::ImageRegionIterator<MukImage>( img, nextRegion);
    while ( ! src.IsAtEnd())
    {
      target.Set(src.Get());
      ++src;
      ++target;
    }
    rgLateralSoftTissue->SetSeed(seedIter.GetIndex());
    rgLateralSoftTissue->SetLower(1);
    rgLateralSoftTissue->SetUpper(1);
    rgLateralSoftTissue->SetInput(img);
    //rg->GetOutput()->SetRequestedRegion(nextRegion); // does not work
    rgLateralSoftTissue->Update();
    return rgLateralSoftTissue->GetOutput();
  }

  /**
  */
  MukImage::Pointer SoftTissueClassification::computeLowerJawSoftTissue(const ROI& subregion, MukImage* inputImg)
  {
    auto ctImage = make_itk<MukImage>();
    ctImage->Graft( static_cast<MukImage*>( this->GetInputs()[0].GetPointer() ));
    auto eacImage = make_itk<MukImage>();
    eacImage->Graft( static_cast<MukImage*>( this->GetInputs()[2].GetPointer() ));

    auto rgPosteriorMedialSoftTissue = make_itk<itk::ConnectedThresholdImageFilter<MukImage, MukImage>>();
    auto img = make_itk<MukImage>();
    img->SetRegions(ctImage->GetRequestedRegion());
    img->SetOrigin(ctImage->GetOrigin());
    img->SetDirection(ctImage->GetDirection());
    img->SetSpacing(ctImage->GetSpacing());
    img->Allocate();
    img->FillBuffer(0);
    // perform a seed based region growing from somewhere at the far lateral far and anterior side
    // boundaries are defined by the most posterior part of the external auditory canal
    itk::ImageRegion<3u> nextRegion;
    auto lower = subregion.GetIndex();
    auto upper = subregion.GetUpperIndex();
    lower[2] = 0.5*(lower[2] + upper[2]);
    nextRegion.SetIndex(lower);
    upper[1] = nextRegion.GetIndex()[1];
    auto boundaryIter = itk::ImageRegionConstIterator<MukImage>( eacImage, subregion );
    while ( ! boundaryIter.IsAtEnd() )
    {
      if (boundaryIter.Get())
      {
        const auto idx = boundaryIter.GetIndex();
        if (idx[1] > upper[1])
          upper[1] = idx[1];
      }
      ++boundaryIter;
    }
    nextRegion.SetUpperIndex(upper);
    const auto size = img->GetLargestPossibleRegion().GetSize();
    auto seedIter = itk::ImageSliceConstIteratorWithIndex<MukImage> (inputImg, nextRegion );
    seedIter.SetFirstDirection(1);
    seedIter.SetFirstDirection(2);
    bool notFound = true;
    {
      seedIter.GoToReverseBegin();
      while ( (! seedIter.IsAtReverseEnd()) )//&& notFound)
      {
        while ( (! seedIter.IsAtEndOfSlice()) )//&& notFound)
        {
          while ( (! seedIter.IsAtEndOfLine()) )//&& notFound)
          {
            if (seedIter.Get())
            {
              notFound = false;
            }
            ++seedIter;
          }
          seedIter.NextLine();
        }
        seedIter.PreviousSlice();
      }
      if (notFound)
      {
        throw MUK_EXCEPTION_SIMPLE("Warning: Seed Iterator out of bounds");
      }
    }
    // fill img3 with the segmentation
    auto src = itk::ImageRegionConstIterator<MukImage>( inputImg, nextRegion);
    auto target = itk::ImageRegionIterator<MukImage>( img, nextRegion);
    while ( ! src.IsAtEnd())
    {
      target.Set(src.Get());
      ++src;
      ++target;
    }
    rgPosteriorMedialSoftTissue->SetSeed(seedIter.GetIndex());
    rgPosteriorMedialSoftTissue->SetLower(1);
    rgPosteriorMedialSoftTissue->SetUpper(1);
    rgPosteriorMedialSoftTissue->SetInput(img);
    rgPosteriorMedialSoftTissue->Update();
    return rgPosteriorMedialSoftTissue->GetOutput();
  }

  /**
  */
  void SoftTissueClassification::computeUpperMedialRegions(const ROI& subregion, MukImage* imgMedialSoftTissue, Regions& medialRegions, Regions& lateralRegions, MukImage::Pointer& partialResult)
  {
    auto layerGrowing2 = make_itk<LayerGrowing>();
    size_t N(0);
    {
      auto not = make_itk<itk::NotImageFilter<MukImage, MukImage>>();
      not->SetInput(imgMedialSoftTissue);
      // the next to filters will try to remove small connecting tubes like the carotid artery and the jugular vein via tresholding on a distance map
      using DistanceFilter = itk::DanielssonDistanceMapImageFilter<MukImage, MukImage>;
      auto distMap = make_itk<DistanceFilter>();
      distMap->SetInput(not->GetOutput());

      auto thresh = make_itk<itk::BinaryThresholdImageFilter<MukImage, MukImage>>();
      thresh->SetLowerThreshold(mDistance);
      thresh->SetInput(distMap->GetOutput());

      auto conn = make_itk<itk::ConnectedComponentImageFilter<MukImage, MukImage>>();
      conn->SetInput(thresh->GetOutput());
      conn->Update();
      N = conn->GetObjectCount();

      auto layerGrowing1 = make_itk<LayerGrowing>();
      layerGrowing1->SetInput(conn->GetOutput());
      layerGrowing1->SetLabeledImage(distMap->GetOutput());
      layerGrowing1->Modified();
      layerGrowing1->Update();

      layerGrowing2->SetInput(layerGrowing1->GetOutput());
      layerGrowing2->SetLabeledImage(imgMedialSoftTissue);
      layerGrowing2->Modified();
      layerGrowing2->Update();
      partialResult = layerGrowing2->GetOutput();
    }
    // we have N potential regions
    Regions regions(N);
    extractRegionsFromLabelImage(regions, layerGrowing2->GetOutput());

    const auto N_tmp = regions.size();
    regions.erase(std::remove_if(regions.begin(), regions.end(), [&] (const auto& v) { return v.size() < mMinimalNumberOfVoxels; }), regions.end());
    // the region is a medial one if there is a medial voxel in the upper 10 layers
    std::vector<bool> isMedial(regions.size(), false);
    const auto minZ = subregion.GetUpperIndex()[2]-20;
    const auto maxX = subregion.GetIndex()[0]+20;
    for (size_t i(0); i<regions.size(); ++i)
    {
      for (const auto& idx : regions[i])
      {
        if ( idx[2] >= minZ
          && idx[0] < maxX )
        {
          isMedial[i] = true;
          break;
        }
      }
    }
    for (size_t i(regions.size()); i>0; --i)
    {
      if (isMedial[i-1])
      {
        medialRegions.push_back(std::move(regions[i-1]));
      }
      else
      {
        lateralRegions.push_back(std::move(regions[i-1]));
      }
    }
  }

  /**
  */
  void SoftTissueClassification::extractBrain(const ROI& subregion, MukImage* imgSoftTissue, const MukImage* imgAir, Regions& medialRegions, Regions& lateralRegions, MukImage::Pointer& partialResult)
  {
    auto zMax = subregion.GetUpperIndex()[2];
    auto zMin = subregion.GetIndex()[2];    
    using SliceImage = itk::Image<MukPixel, 2>;
    auto brainImage = make_itk<MukImage>();
    allocateFromSrc(imgSoftTissue, brainImage);
    //// prepare region
    ROI extractRegion = subregion;
    auto size = extractRegion.GetSize();
    size[2] = 0;
    extractRegion.SetSize(size);
    // fill medialRegions
    bool anteriorBrainTissueAvailable = true;
    bool posteriorBrainTissueAvailable = true;
    std::vector<SliceImage::IndexType> validAnteriorIndices;
    std::vector<SliceImage::IndexType> validPosteriorIndices;
    SliceImage::IndexValueType minY = 0;
    for (unsigned int idxSlice_(zMax); idxSlice_>zMin; --idxSlice_)
    {
      auto idxSlice = idxSlice_-1;
      itk::Index<3u> index = extractRegion.GetIndex();
      index[2] = idxSlice;
      extractRegion.SetIndex(index);

      auto pExtractInput = make_itk<itk::ExtractImageFilter<MukImage, SliceImage>>();
      pExtractInput->SetDirectionCollapseToSubmatrix();
      pExtractInput->SetInPlace(true);
      pExtractInput->SetInput(imgSoftTissue);
      pExtractInput->SetExtractionRegion(extractRegion);
      pExtractInput->Update();

      auto conn = make_itk<itk::ConnectedComponentImageFilter<SliceImage, SliceImage>>();
      conn->SetInput(pExtractInput->GetOutput());
      conn->Update();

      auto it = itk::ImageLinearConstIteratorWithIndex<SliceImage>(conn->GetOutput(), conn->GetOutput()->GetBufferedRegion());
      it.SetDirection(0);
      std::map<MukPixel, unsigned int> brainLabels;
      if (anteriorBrainTissueAvailable)
      {
        it.GoToBeginOfLine(); // thats the most medial and anterior point
        std::vector<SliceImage::IndexType> validIndices;
        const auto width = conn->GetOutput()->GetLargestPossibleRegion().GetSize()[1];
        for (size_t i(0); i<0.25*width; ++i)
        {
          auto val = it.Get();
          if (val)
          {
            ++brainLabels[val];
            if (idxSlice==zMax)
            {
              validAnteriorIndices.push_back(it.GetIndex());
              validIndices.push_back(it.GetIndex());
            }
            else
            {
              validIndices.push_back(it.GetIndex());
            }
          }
          ++it;
        }
        if (brainLabels.empty()
          || validIndices.size() < 0.5*validAnteriorIndices.size())
        {
          anteriorBrainTissueAvailable = false;
        }
      }
      it.GoToReverseBegin();
      if (posteriorBrainTissueAvailable)
      {
        it.GoToBeginOfLine(); // thats the most lateral and posterior point
        std::vector<SliceImage::IndexType> validIndices;
        const auto width = conn->GetOutput()->GetLargestPossibleRegion().GetSize()[1];
        for (size_t i(0); i<0.25*width; ++i)
        {
          auto val = it.Get();
          if (val)
          {
            ++brainLabels[val];
            if (idxSlice==zMax)
            {
              validPosteriorIndices.push_back(it.GetIndex());
              validIndices.push_back(it.GetIndex());
            }
            else
            {
              validIndices.push_back(it.GetIndex());
            }
          }
          ++it;
        }
        if (brainLabels.empty()
          || validIndices.size() < 0.5*validPosteriorIndices.size())
        {
          posteriorBrainTissueAvailable = false;
        }
      }
      Region v1,v2;
      auto currentMinY = conn->GetOutput()->GetLargestPossibleRegion().GetUpperIndex()[1];
      for (it.GoToBegin(); !it.IsAtEnd(); it.NextLine())
      {
        for (it.GoToBeginOfLine(); !it.IsAtEndOfLine(); ++it)
        {
          auto val = it.Get();
          if (val)
          {
            MukImage::IndexType idx;
            idx[0] = it.GetIndex()[0];
            idx[1] = it.GetIndex()[1];
            idx[2] = idxSlice;
            if ( idx[1] >= minY && std::any_of(brainLabels.begin(), brainLabels.end(), [&] (const auto& pair) { return pair.first == val; }))
            {
              v1.push_back(idx);
              if (idx[1] < currentMinY)
                currentMinY = idx[1];
            }
            else
            {
              v2.push_back(idx);
            }
          }
        }
      }
      medialRegions.push_back(v1);
      lateralRegions.push_back(v2);
      minY = currentMinY;
    }
  }

  /**
  */
  MukImage* SoftTissueClassification::getOutput1()
  {
    return dynamic_cast<MukImage*>( this->ProcessObject::GetOutput(0) );
  }

  /**
  */
  MukImage* SoftTissueClassification::getOutput2()
  {
    return dynamic_cast<MukImage*>( this->ProcessObject::GetOutput(1) );
  }

  /**
  */
  MukImage* SoftTissueClassification::getOutput3()
  {
    return dynamic_cast<MukImage*>( this->ProcessObject::GetOutput(2) );
  }

  /**
  */
  MukImage* SoftTissueClassification::getOutput4()
  {
    return dynamic_cast<MukImage*>( this->ProcessObject::GetOutput(3) );
  }
}
}