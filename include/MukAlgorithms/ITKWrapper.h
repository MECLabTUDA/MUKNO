#pragma once
#include "AlgorithmWrapper.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    template<typename InputType, typename OutputType=InputType>
    class MUK_ALGO_API ItkImageToImageWrapper : public AlgorithmWrapper
    {
      public:
        ItkImageToImageWrapper() {}

      public:
        virtual void  setInput(unsigned int portId, void* pDataType)
        {
          auto pData = toDataType<InputType>(pDataType);
          mpFilter->SetInput(pData);
        }

        virtual void*  getOutput(unsigned int portId)         { return static_cast<void*>(mpFilter->GetOutput()); }
        virtual void   update()                               { mpFilter->UpdateLargestPossibleRegion(); }

      protected:
        itk::SmartPointer<itk::ImageToImageFilter<InputType, OutputType>> mpFilter;
    };

    /**
    */
    template<typename OutputType>
    class MUK_ALGO_API ItkImageSourceWrapper : public AlgorithmWrapper
    {
      public:
        virtual void  setInput(unsigned int portId, void* pDataType)
        {
          throw MUK_EXCEPTION("Algorithm has no input port", name());
        }

        virtual void*  getOutput(unsigned int portId)        { return static_cast<void*>(mpFilter->GetOutput()); }
        virtual void   update()                              { mpFilter->UpdateLargestPossibleRegion(); }

      protected:
        itk::SmartPointer< itk::ImageSource<OutputType> > mpFilter;
    };

    /**
    */
    template<typename InputType>
    class MUK_ALGO_API ItkProcessObjectWrapper : public AlgorithmWrapper
    {
      public:
        virtual void* getOutput(unsigned int portId)
        {
          throw MUK_EXCEPTION("Algorithm has no output port", name());
        }

        virtual void update()
        {
          mpFilter->UpdateLargestPossibleRegion();
        }

      protected:
        itk::SmartPointer<itk::ProcessObject> mpFilter;
    };
  }
}