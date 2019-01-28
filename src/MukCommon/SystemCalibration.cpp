#include "private/muk.pch"
#include "SystemCalibration.h"
#include "muk_common.h"

#include <vtkSmartPointer.h>
#include <vtkMetaImageReader.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

namespace gris
{
namespace muk
{
  namespace fs = boost::filesystem;

  /**
  */
  CTCalibration::CTCalibration()
    : mDimSize(std::numeric_limits<size_t>::quiet_NaN())
    , mElementSpacing(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /**
  */
  CTCalibration::~CTCalibration()
  {
  }

  /**
  */
  void CTCalibration::readData(const std::string& filename)
  {    
    fs::path path = filename;
    if (0 == path.extension().string().compare("mhd"))
    {
      auto tmp = boost::format("%s: cannot read File '%s'. Checked for ending '.mhd'?  ") % __FUNCTION__ % path.c_str();
      throw std::exception(tmp.str().c_str());
    }

    DefVtk(vtkMetaImageReader, reader);    
    reader->SetFileName(path.string().c_str());
    reader->Update();
    double* spacing = reader->GetDataSpacing();
    mElementSpacing = Vec3d(spacing[0], spacing[1], spacing[2]);
    int* extent     = reader->GetDataExtent(); // thats minx maxx, miny maxy, minz maxz.
    //mDimSize        = Vec3u(extent[0], extent[1], extent[2]);
    int dimX = extent[1] - extent[0] + 1;
    int dimY = extent[3] - extent[2] + 1;
    int dimZ = extent[5] - extent[4] + 1;
    mDimSize = Vec3d(dimX, dimY, dimZ);
  }

  /**
  */
  bool CTCalibration::dataCoincides(const std::string& filename) const
  {    
    fs::path path = filename;
    if (0 == path.extension().string().compare("mhd"))
    {
      auto tmp = boost::format("%s: cannot read File '%s'. Checked for ending '.mhd'?  ") % __FUNCTION__ % path.c_str();
      throw std::exception(tmp.str().c_str());
    }

    DefVtk(vtkMetaImageReader, reader);
    reader->SetFileName(path.string().c_str());
    reader->Update();
    double* spacing_ = reader->GetDataSpacing();
    Vec3d spacing(spacing_[0], spacing_[1], spacing_[2]);
    int* extent = reader->GetDataExtent();
    //Vec3u dim   = Vec3u(extent[0], extent[1], extent[2]);
    int dimX = extent[1] - extent[0] + 1;
    int dimY = extent[3] - extent[2] + 1;
    int dimZ = extent[5] - extent[4] + 1;
    auto dim = Vec3d(dimX, dimY, dimZ);
    if ( (! mElementSpacing.hasNaN()) && mElementSpacing != spacing )   // variable has already been set + unequal
    {
      return false;
    }
    if ( (! mDimSize.hasNaN()) && mDimSize != dim ) // variable has already been set + unequal
    {
      return false;
    }
    return true;
  }

  /*const char* CTCalibration::print()
  {}*/

}
}
