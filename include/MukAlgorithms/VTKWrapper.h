#pragma once
#include "AlgorithmWrapper.h"

#include <vtkSmartPointer.h>
#include <vtkPolyDataAlgorithm.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_ALGO_API VtkWrapper : public AlgorithmWrapper
    {
      public:
        VtkWrapper() {}

      public:
        virtual void*  getOutput(unsigned int portId)       { return static_cast<void*>(mpFilter->GetOutput()); }
        virtual void   update()                             { mpFilter->Update(); }

      protected:
        vtkSmartPointer<vtkPolyDataAlgorithm> mpFilter;
    };
  }
}