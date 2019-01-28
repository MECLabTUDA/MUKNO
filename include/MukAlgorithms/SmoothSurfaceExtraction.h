#pragma once

#include "AlgorithmWrapper.h"

#include <vtkSmartPointer.h>
class vtkPolyDataNormals;

namespace gris
{
  namespace muk
  {
    /** \brief A improved version of the pdmlib's surface-extraction
    */
    class MUK_ALGO_API SmoothSurfaceExtraction : public AlgorithmWrapper
    {
      public:
        SmoothSurfaceExtraction();

      public:
        static  const char* s_name()       { return "SmoothSurfaceExtraction"; };
        virtual const char* name()   const { return s_name(); }

      public:
        virtual void   setInput(unsigned int portId, void* pDataType);
        virtual void*  getOutput(unsigned int portId);
        virtual void   update();

      public:
        void setSurfaceValue(double val)  { mSurfaceValue = val; }
        void setSigma(const Vec3d& val)   { mSigma = val; }
        void setClosingKernel(const Vec3d& val) { mClosingKernel = val; }
        void setLabel(MukPixel val)       { mLabelValue = val; }
        void setPadding(int val)          { mPaddingValue = val; }
        void setAreaThreshold(int val)    { mAreaTreshold = val; }

        double        getSurfaceValue()   const { return mSurfaceValue; }
        const Vec3d&  getSigma()          const { return mSigma; }
        const Vec3d&  getClosingKernel()  const { return mClosingKernel; }
        MukPixel      getLabel()          const { return mLabelValue; }
        int           getPadding()        const { return mPaddingValue; }
        int           getAreaThreshold()  const { return mAreaTreshold; }

      private:
        ImageInt3D*    mpInputImage;
        vtkSmartPointer<vtkPolyDataNormals> mpOutputMesh;

        // parameter
        double    mSurfaceValue;
        Vec3d     mSigma;
        Vec3d     mSpacing;
        Vec3d     mClosingKernel;
        MukPixel  mLabelValue;
        int       mPaddingValue;
        int       mAreaTreshold;
    };
  }
}