#pragma once

#include "VtkWrapper.h"
#include <vtkPolyData.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_ALGO_API PolyDataReader : public VtkWrapper
    {
      public:
        PolyDataReader();

      public:
        static  const char* s_name()       { return "PolyDataReader"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void*  getOutput(unsigned int portId)       { return static_cast<void*>(mpOutput); }
        virtual void   setInput(unsigned int portId, void* pDataType) {}
        virtual size_t sizeInputs() const  { return 0; }
        virtual size_t sizeOutputs() const { return 1; }
        virtual void   update();

      private:
        vtkSmartPointer<vtkPolyData> mpOutput;
        std::string mStringCache;
        bool mModified;
    };
  }
}
