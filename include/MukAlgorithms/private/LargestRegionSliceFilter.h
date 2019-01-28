#pragma once

#include "MukImaging\ImageInt3D.h"
#include "itkImageToImageFilter.h"


namespace gris
{
  namespace muk
  {
    class LargestRegionSliceFilter : public itk::ImageToImageFilter<ImageInt2D, ImageInt2D>
    {
      public:
      /** Standard class typedefs. */
      typedef LargestRegionSliceFilter   Self;
      typedef ImageToImageFilter< ImageInt2D, ImageInt2D > Superclass;
      typedef itk::SmartPointer< Self >                Pointer;
      itkNewMacro(Self);												/** Method for creation through the object factory. */
      itkTypeMacro(ImageFilter, ImageToImageFilter);		/** Run-time type information (and related methods). */

      protected:
      virtual void GenerateInputRequestedRegion();

      private:
      virtual void GenerateData();

      private:
		  LargestRegionSliceFilter() {}
		  LargestRegionSliceFilter(const Self &); //purposely not implemented
      ~LargestRegionSliceFilter() {}

      void operator=(const Self &);  //purposely not implemented
    };
  }
}