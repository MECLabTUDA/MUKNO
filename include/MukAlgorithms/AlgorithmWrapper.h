#pragma once

#include "MukCommon/MukException.h"
#include "MukImaging/MukImage.h"
#include "gstd/DynamicProperty.h"
#include "muk_algorithms_api.h"
#include <itkProcessObject.h>
#include <itkImageToImageFilter.h>
#include <itkImageSource.h>

class vtkImageData;
class vtkPolyData;

namespace gris
{
  namespace muk
  {
    enum EnDataType
    {
      enImageInt2D,
      enImageInt3D,
      enImageFloat3D,
      enGradientImage3D,
      enVtkImage,
      enVtkMesh,
      enVtkPolyData,
      enImageTypeSize,
    };

    enum MUK_ALGO_API EnDisplayType
    {
      enDisplayImage2D = 0,
      enDisplayImage3D,
      enDisplayOverlay2D,
      enDisplayOverlay3D,
      enDisplayPolyData,
      enDisplayTypeSize
    };

    static const std::array<std::string, enDisplayTypeSize> DisplayTypeNames = 
    {
      "Image2D",
      "Image3D",
      "Overlay2D",
      "Overlay3D",
      "PolyData",
    };


    template<typename T>
    auto* toDataType(void* pData)
    {
      return static_cast<T*>(pData);
    }

    template<enum EnDataType>
    auto* toDataType(void* pData)
    {
      return pData;
    }

    template<> MUK_ALGO_API auto* toDataType<enImageInt3D>(void* pData);
    template<> MUK_ALGO_API auto* toDataType<enImageFloat3D>(void* pData);
    template<> MUK_ALGO_API auto* toDataType<enVtkImage>(void* pData);
    ////template<> auto* toDataType<enVtkMesh>(void* pData)         { return static_cast<vtk*>(pData); }
    template<> MUK_ALGO_API auto* toDataType<enVtkPolyData>(void* pData);

    /**
    */
    class MUK_ALGO_API AlgorithmWrapper : public gris::gstd::DynamicProperty
    {
      public:
        AlgorithmWrapper();

      public:
        virtual const char* name() const = 0;

        virtual void   setInput (unsigned int portId, void* pDataType) = 0;
        virtual void*  getOutput(unsigned int portId) = 0;
        virtual void   update() = 0;

    public:
        size_t sizeInputs()  const { return mInputPortTypes.size(); }
        size_t sizeOutputs() const { return mOutputPortTypes.size(); }

    public:
        void                setAlias(const std::string& name)         { mAlias = name; }
        const std::string&  getAlias()                          const { return mAlias; }
        EnDataType          getInputType (unsigned int portId)  const { return mInputPortTypes[portId]; }
        EnDataType          getOutputType(unsigned int portId)  const { return mOutputPortTypes[portId]; }
        void                setDisplayType(EnDisplayType val)         { mDspType = val; }
        EnDisplayType       getDisplayType() 			              const { return mDspType; }
		    
      protected:
        unsigned int  mIdx;
        std::string   mAlias; /// e.g. to give an segmentation algorithm an alias for the label / organ it segments
        std::vector<EnDataType> mInputPortTypes;
        std::vector<EnDataType> mOutputPortTypes;
        EnDisplayType mDspType;
    };
  } // namespace muk
} // namespace gris

namespace std
{
  MUK_ALGO_API std::ostream& operator<< (std::ostream& os, const gris::muk::EnDisplayType& obj);
  MUK_ALGO_API std::istream& operator>> (std::istream& is, gris::muk::EnDisplayType& obj);
}
