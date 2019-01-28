#pragma once
#include "VTKWrapper.h"

#include <itkImageToVtkImageFilter.h>

#include <vtkMarchingCubes.h>

class vtkImageData;

namespace gris
{
  namespace muk
  {
    /** \brief Extracts a triangle mesh from a binary image based on an interval [0, value]. Remeshing can be added optionally.

      A Remeshing value of 0 decodes "No remeshing".
    */
    class MUK_ALGO_API MarchingCubes : public VtkWrapper
    {
      public:
		    MarchingCubes();

      public:
        virtual void  setInput(unsigned int portId, void* pDataType);
        virtual void* getOutput(unsigned int portId);
        virtual void  update();

      public:
        static  const char* s_name()       { return "MarchingCubes"; };
        virtual const char* name()   const { return s_name(); }

      private:
        struct Impl;

      private:
        std::unique_ptr<Impl> mp;
    };

  } // namespace muk
} // gris