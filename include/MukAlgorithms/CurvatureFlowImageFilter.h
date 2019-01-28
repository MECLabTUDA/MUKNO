#pragma once

#include "ITKWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Class to wrapp itk::CurvatureFlowImageFilter
    */
    class MUK_ALGO_API CurvatureFlowImageFilter : public AlgorithmWrapper
    {
      public:
        CurvatureFlowImageFilter();

      public:
        virtual void  setInput(unsigned int portId, void* pDataType)
        {
          auto pData = toDataType<ImageInt3D>(pDataType);
          curvatureFilter->SetInput(pData);
          outputFilter->SetInput(curvatureFilter->GetOutput());
        }

        virtual void* getOutput(unsigned int portId)
        {
          return static_cast<void*>(outputFilter->GetOutput());
        }

        virtual void update()
        {
          outputFilter->Update();
        }

        static  const char* s_name() { return "CurvatureFlowImageFilter"; };
        virtual const char* name()   const { return s_name(); }

      protected:
        itk::SmartPointer<itk::ImageToImageFilter<ImageInt3D, ImageFloat3D>> curvatureFilter;
        itk::SmartPointer<itk::ImageToImageFilter<ImageFloat3D, ImageInt3D>> outputFilter;
    };
  } // namespace muk
} // gris