#pragma once
#include "MukImaging\MukImage.h"

#include <itkImageToImageFilter.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    class LargestRegionFilter : public itk::ImageToImageFilter<ImageInt3D, ImageInt3D>
    {
      public:
        /** Standard class typedefs. */
        typedef LargestRegionFilter   Self;
        typedef ImageToImageFilter< ImageInt3D, ImageInt3D > Superclass;
        typedef itk::SmartPointer< Self >                Pointer;
        itkNewMacro(Self);												/** Method for creation through the object factory. */
        itkTypeMacro(ImageFilter, ImageToImageFilter);		/** Run-time type information (and related methods). */

      protected:
        virtual void GenerateInputRequestedRegion();

      private:
        virtual void GenerateData();

      private:
		    LargestRegionFilter() {}
		    LargestRegionFilter(const Self &); //purposely not implemented
        ~LargestRegionFilter() {}

        void operator=(const Self &);  //purposely not implemented
    };
  }
}