#pragma once
#include "AlgorithmWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Creates a Label image based on the interior of a mesh.

      input port 0: a mesh
      input port 1: a 3D image
    */
    class MUK_ALGO_API MeshToImageFilter : public AlgorithmWrapper
    {
      public:
        MeshToImageFilter();

      public:
        static  const char* s_name()       { return "MeshToImage"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput(unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);
        virtual void   update();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
