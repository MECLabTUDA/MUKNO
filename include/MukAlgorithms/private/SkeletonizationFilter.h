/*==================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/

#pragma once

#include "itkImageToImageFilter.h"
#include "itkImage.h"
#include <itkVectorImage.h>

namespace itk
{
  template < class TInputImage, class TOutputImage >
  class SkeletonizationFilter : public ImageToImageFilter<TInputImage, TOutputImage>
  {
    public:
      using InputImageType  = TInputImage;
      using VectorType      = itk::CovariantVector<int,3>;
      using VectorImageType = itk::Image<VectorType, 3>;
      using OutputImageType = TOutputImage;
      using GradientImageType = itk::VectorImage<int, 3>;
      using Self            = SkeletonizationFilter;
      using Superclass      = ImageToImageFilter<InputImageType, OutputImageType>;
      using Pointer         = SmartPointer<Self>;
      using ConstPointer    = SmartPointer<const Self>;

      itkFactorylessNewMacro(Self)
      itkCloneMacro(Self)

    public:
      virtual void GenerateData();

    public:
      GradientImageType::Pointer GetGradientImage();
      VectorImageType::Pointer   GetVectorImage()     { return m_DirectionImage; }

    protected:
      SkeletonizationFilter();
      virtual ~SkeletonizationFilter();

    protected:
      VectorImageType::Pointer m_DirectionImage;
      int round(float x)
      {
        if (x>0.0) return ((int) (x+0.5));
        else       return ((int) (x-0.5));
      }

    protected:
  };

}

#include "private/SkeletonizationFilter.hxx"