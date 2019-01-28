#pragma once
#include "VtkWrapper.h"

namespace gris
{
  namespace muk
  {
    /** \brief Remeshes a given Mesh considering number of points or resolution.

      uses the ADVC library
    */
    class MUK_ALGO_API Remesher : public VtkWrapper
    {
      public:
        Remesher();

      public:
        static  const char* s_name()        { return "Remesher"; };
        virtual const char* name()   const  { return s_name(); }

      public:
        virtual void   setInput (unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);

        virtual void   update();

      private:
        bool hasBadInput();

      private:
        vtkPolyData* mpInput;
        vtkSmartPointer<vtkPolyData> mpOutput;
        size_t mNumberOfSamples;
        double mGradation;
        int    mSubsamplingThreshold;
        double mSplitLongMeshesRatio;
    };

  } // namespace muk
} // gris