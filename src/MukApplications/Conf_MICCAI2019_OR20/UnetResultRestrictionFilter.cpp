#include "private/muk.pch"
#include "private/UnetResultRestrictionFilter.h"

#include "MukAlgorithms/AlgorithmFactory.h"
#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkBinaryThresholdImageFilter.h>
#include <itkConnectedComponentImageFilter.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageRegionIterator.h>
#include <itkLabelShapeKeepNObjectsImageFilter.h>
#include <itkRescaleIntensityImageFilter.h>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(UnetResultRestrictionFilter);

    /**
    */
    struct UnetResultRestrictionFilter::Impl
    {
      Impl()
      {
        pOutput = make_itk<ImageInt3D>();
      }

      ImageInt3D* pInput;
      ImageInt3D::Pointer pOutput;
      bool modified = false;
    };

    /**
    */
    UnetResultRestrictionFilter::UnetResultRestrictionFilter()
      : mp(std::make_unique<Impl>())
    {
      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enImageInt3D);
      mDspType = enDisplayImage3D;
    }

    /**
    */
    UnetResultRestrictionFilter::~UnetResultRestrictionFilter()
    {
    }

    /**
    */
    void UnetResultRestrictionFilter::setInput (unsigned int portId, void* pDataType)
    {
      auto* p = static_cast<ImageInt3D*>(pDataType);
      mp->pInput = p;
      mp->modified = true;
    }

    /**
    */
    void* UnetResultRestrictionFilter::getOutput(unsigned int portId)
    {
      return mp->pOutput.GetPointer();
    }

    /**
    */
    void UnetResultRestrictionFilter::update()
    {
      bool mTimeUnchanged = mp->pInput->GetMTime() < mp->pOutput->GetMTime();
      if (!mp->modified && mTimeUnchanged)
        return;

      allocateFromSrc(*mp->pInput, *mp->pOutput, true, 0);

      const auto Max_Label = 10u; // Mukno labels are 1 (ICA) to 10 (EAC)
      for (int i(1); i<= Max_Label; ++i)
      {
        auto thresh = make_itk<itk::BinaryThresholdImageFilter<ImageInt3D,ImageInt3D>>();
        thresh->SetLowerThreshold(i);
        thresh->SetUpperThreshold(i);
        thresh->SetOutsideValue(0);
        thresh->SetInsideValue(i);
        thresh->SetInput(mp->pInput);

        /*auto conn = make_itk<itk::ConnectedComponentImageFilter<ImageInt3D, ImageInt3D>>();
        conn->SetInput(thresh->GetOutput());
        conn->Update();

        LOG_LINE << "Number of objects: " << conn->GetObjectCount();
        using LabelLargest = itk::LabelShapeKeepNObjectsImageFilter< ImageInt3D >;
        auto labelFilter = make_itk<LabelLargest>();
        labelFilter->SetInput( conn->GetOutput() );
        labelFilter->SetBackgroundValue( 0 );
        labelFilter->SetNumberOfObjects( 1 );
        labelFilter->SetAttribute( LabelLargest::LabelObjectType::NUMBER_OF_PIXELS);*/

        auto rescaleFilter = make_itk<itk::RescaleIntensityImageFilter< ImageInt3D, ImageInt3D>>();
        rescaleFilter->SetOutputMinimum(0);
        rescaleFilter->SetOutputMaximum(i);
        //rescaleFilter->SetInput(labelFilter->GetOutput());
        rescaleFilter->SetInput(thresh->GetOutput());
        rescaleFilter->Update();

        auto itIn  = itk::ImageRegionConstIterator<ImageInt3D>(rescaleFilter->GetOutput(), thresh->GetOutput()->GetBufferedRegion());
        auto itOut = itk::ImageRegionIterator<ImageInt3D>(mp->pOutput, mp->pOutput->GetBufferedRegion());

        while (!itIn.IsAtEnd())
        {
          if (itIn.Get())
          {
            itOut.Set(i);
          }
          ++itIn;
          ++itOut;
        }
      }
      mp->modified = false;
    }
  }
}