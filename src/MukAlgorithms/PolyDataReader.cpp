#include "private/muk.pch"
#include "PolyDataReader.h"
#include "AlgorithmFactory.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include <vtkPolyDataReader.h>
#include <vtkOBJReader.h>

#include <boost/filesystem.hpp>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(PolyDataReader);

    /**
    */
    PolyDataReader::PolyDataReader()
      : mModified(false)
    {
      mDspType = enDisplayPolyData;
      mOutputPortTypes.push_back(enVtkPolyData);

      mpOutput = make_vtk<vtkPolyData>();

      declareProperty<std::string>("Filename"
        , [&] (const std::string& fn) { mStringCache = fn; mModified = true; }
        , [&] ()                      { return mStringCache; });
    }
    
    /**
    */
    void PolyDataReader::update()
    {
      if (! mModified)
        return;

      namespace fs = boost::filesystem;
      fs::path p(mStringCache);
      if ( ! fs::is_regular_file(mStringCache))
      {
        throw MUK_EXCEPTION("File does not exist", mStringCache.c_str());
      }
      if (p.extension().string() == ".vtk")
      {
        auto reader = make_vtk<vtkPolyDataReader>();
        reader->SetFileName(mStringCache.c_str());
        reader->Update();
        mpOutput->DeepCopy(reader->GetOutput());
      }
      else if (p.extension().string() == ".obj")
      {
        auto reader = make_vtk<vtkOBJReader>();
        reader->SetFileName(mStringCache.c_str());
        reader->Update();
        mpOutput->DeepCopy(reader->GetOutput());
      }
      mpOutput->Modified();
    }
  }
}
