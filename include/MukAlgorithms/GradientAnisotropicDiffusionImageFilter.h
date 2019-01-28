#pragma once

#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::GradientAnisotropicDiffusionImageFilter
    */
    class MUK_ALGO_API GradientAnisotropicDiffusionImageFilter : public AlgorithmWrapper
    {
      public:
        GradientAnisotropicDiffusionImageFilter();

      public:
        static  const char* s_name() { return "GradientAnisotropicDiffusionImageFilter"; };
        virtual const char* name()   const { return s_name(); }


      public:
        virtual void  setInput(unsigned int portId, void* pDataType)
        {
          auto pData = toDataType<ImageInt3D>(pDataType);
          smoothingFilter->SetInput(pData);
          outputFilter->SetInput(smoothingFilter->GetOutput());
        }

        virtual void* getOutput(unsigned int portId)
        {
          return static_cast<void*>(outputFilter->GetOutput());
        }

        virtual void update()
        {
          outputFilter->Update();
        }

      protected:
        itk::SmartPointer<itk::ImageToImageFilter<ImageInt3D, ImageFloat3D>> smoothingFilter;
        itk::SmartPointer<itk::ImageToImageFilter<ImageFloat3D, ImageInt3D>> outputFilter;
    };

  } // namespace muk
} // gris