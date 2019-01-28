#pragma once
#include "VTKWrapper.h"

class vtkDecimatePro;

namespace gris
{
  namespace muk
  {
    /** \brief Creates a grayscale image by thresholding at a lower and/or upper limit.
    */
    class MUK_ALGO_API MeshReduction : public VtkWrapper
    {
      public:
        static  const char* s_name()       { return "MeshReduction"; };
        virtual const char* name()   const { return s_name(); }

      public:
        MeshReduction();

      public:
        virtual void  setInput(unsigned int portId, void* pDataType);
        virtual void* getOutput(unsigned int portId);
        virtual void  update();

      public:
        void   setNumberofPoints(size_t N);
        size_t getNumberofPoints()         const { return mNumberOfPoints; }

      private:
        void removeSmallSubMeshes();

      private:
        vtkPolyData*                 mpInput  = nullptr;
        vtkSmartPointer<vtkPolyData> mpOutput = nullptr;
        size_t mNumberOfPoints;
        size_t mMinimumPoints;
        vtkSmartPointer<vtkPolyData> mpWorker;
    };

  } // namespace muk
} // gris