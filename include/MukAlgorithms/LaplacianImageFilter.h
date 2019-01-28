#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::CannyEdgeDetectionImageFilter
    */
    class MUK_ALGO_API LaplacianImageFilter : public AlgorithmWrapper
    {
      public:
        LaplacianImageFilter();

      public:
        virtual void  setInput(unsigned int portId, void* pDataType)
        {
          auto pData = toDataType<ImageInt3D>(pDataType);
          inputFilter->SetInput(pData);
          laplacianFilter->SetInput(inputFilter->GetOutput());
          outputFilter->SetInput(laplacianFilter->GetOutput());
        }

        virtual void* getOutput(unsigned int portId)
        {
          return static_cast<void*>(outputFilter->GetOutput());
        }

        virtual void update()
        {
          outputFilter->Update();
        }

        static  const char* s_name() { return "LaplacianImageFilter"; };
        virtual const char* name()   const { return s_name(); }

      protected:
        itk::SmartPointer<itk::ImageToImageFilter<ImageInt3D, ImageFloat3D>> inputFilter;
        itk::SmartPointer<itk::ImageToImageFilter<ImageFloat3D, ImageFloat3D>> laplacianFilter;
        itk::SmartPointer<itk::ImageToImageFilter<ImageFloat3D, ImageInt3D>> outputFilter;
    };

  } // namespace muk
} // gris