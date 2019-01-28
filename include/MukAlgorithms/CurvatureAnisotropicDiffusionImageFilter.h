#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::CurvatureAnisotropicDiffusionImageFilter
    */
    class MUK_ALGO_API CurvatureAnisotropicDiffusionImageFilter : public AlgorithmWrapper
    {
      public:
        CurvatureAnisotropicDiffusionImageFilter();

      public:
        virtual void  setInput(unsigned int portId, void* pDataType)
        {
          auto pData = toDataType<ImageInt3D>(pDataType);
          inputFilter->SetInput(pData);
          diffusionFilter->SetInput(inputFilter->GetOutput());
          outputFilter->SetInput(diffusionFilter->GetOutput());
        }

        virtual void* getOutput(unsigned int portId)
        {
          return static_cast<void*>(outputFilter->GetOutput());
        }

        virtual void update()
        {
          outputFilter->Update();
        }

        static  const char* s_name() { return "CurvatureAnisotropicDiffusionImageFilter"; };
        virtual const char* name()   const { return s_name(); }

      protected:
        itk::SmartPointer<itk::ImageToImageFilter<ImageInt3D, ImageFloat3D>> inputFilter;
        itk::SmartPointer<itk::ImageToImageFilter<ImageFloat3D, ImageFloat3D>> diffusionFilter;
        itk::SmartPointer<itk::ImageToImageFilter<ImageFloat3D, ImageInt3D>> outputFilter;
    };
  } // namespace muk
} // gris