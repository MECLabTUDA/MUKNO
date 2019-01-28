#pragma once

#include "MukImaging\ImageInt3D.h"
#include "itkImageToImageFilter.h"


namespace gris
{
  namespace muk
  {
    class LeastSquaresFittingLineFilter: public itk::ImageToImageFilter<ImageInt3D, ImageInt3D>
    {
      public:
      /** Standard class typedefs. */
      typedef LeastSquaresFittingLineFilter   Self;
      typedef ImageToImageFilter< ImageInt3D, ImageInt3D > Superclass;
      typedef itk::SmartPointer< Self >                Pointer;
      itkNewMacro(Self);												/** Method for creation through the object factory. */
      itkTypeMacro(ImageFilter, ImageToImageFilter);		/** Run-time type information (and related methods). */

      protected:
      virtual void GenerateInputRequestedRegion();

      private:
      virtual void GenerateData();

      private:
		  LeastSquaresFittingLineFilter() {}
		  LeastSquaresFittingLineFilter(const Self &); //purposely not implemented
      ~LeastSquaresFittingLineFilter() {}

      void operator=(const Self &);  //purposely not implemented
    };
  }
}