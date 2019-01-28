#pragma once

#include "muk_common_api.h"
#include "MukVector.h"

namespace gris
{
namespace muk
{
  /**
  */
  enum EnEulerDim
  {
    dimX,
    dimY,
    dimZ
  };

  /**
  */
  class MUK_COMMON_API CTCalibration
  {
    public:
      CTCalibration();
      ~CTCalibration();

    public:
      void          setElementSpacing(const Vec3d& v)         { mElementSpacing = v; }
      const Vec3d&  getElementSpacing()                 const { return mElementSpacing; }
      void          setDimSize(const Vec3d& v)                { mDimSize = v; }
      const Vec3d&  getDimSize()                        const { return mDimSize; }

    public:
      void readData(const std::string& filename);
      bool dataCoincides(const std::string& filename) const;

    public:
      /*double millimeterToSpacing(EnEulerDim dim, double millimeter);
      double spacingToMillimeter(EnEulerDim dim, size_t spacing);*/

    public:
      //const char* print();

    private:      
      Vec3d mElementSpacing;
      Vec3d mDimSize;
  };

  /**
  */
  class MUK_COMMON_API SystemCalibration
  {
    public:
    SystemCalibration()  {};
    ~SystemCalibration() {};

    public:
      void                  setCtCalibration(const CTCalibration& c)        { mCtCalibration = c; }
      const CTCalibration&  getCtCalibration()                        const { return mCtCalibration; }

      //const char* print();
      
    private:
      CTCalibration mCtCalibration;
  };
  
}
}
