#pragma once

#include "VtkWrapper.h"
#include <vtkPolyData.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_ALGO_API MeshSource : public VtkWrapper
    {
      public:
        MeshSource();

      public:
        static  const char* s_name()       { return "MeshSource"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void*  getOutput(unsigned int portId);
        virtual void   setInput(unsigned int portId, void* pDataType) {}
        virtual size_t sizeInputs() const  { return 0; }
        virtual size_t sizeOutputs() const { return 1; }
        virtual void   update();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
