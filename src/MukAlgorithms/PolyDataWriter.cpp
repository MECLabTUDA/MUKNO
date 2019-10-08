#include "private/muk.pch"
#include "PolyDataWriter.h"
#include "AlgorithmFactory.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include <vtkPolyDataWriter.h>

#include <boost/filesystem.hpp>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(PolyDataWriter);

  /**
  */
  struct PolyDataWriter::Impl
  {
    vtkSmartPointer<vtkPolyDataWriter> writer;
    vtkPolyData* input = nullptr;
    std::string filename;
    bool modified = true;
  };

  /**
  */
  PolyDataWriter::PolyDataWriter()
    : mp(std::make_unique<Impl>())
  {
    mInputPortTypes.push_back(enVtkPolyData);
    mDspType   = enDisplayPolyData;
    mp->writer = make_vtk<vtkPolyDataWriter>();

    declareProperty<std::string>("Filename"
      , [&] (const std::string& fn) { mp->filename = fn; mp->modified = true; }
      , [&] ()                      { return mp->filename; });
  }

  /**
  */
  void PolyDataWriter::setInput(unsigned int portId, void* pDataType)
  {
    mp->input = toDataType<vtkPolyData>(pDataType);
    mp->modified = true;
  }

  /**
  */
  void PolyDataWriter::update()
  {
    if (mp->modified)
    {
      mp->writer->SetFileName(mp->filename.c_str());
      mp->writer->SetInputData(mp->input);
      mp->modified = false;
    }

    namespace fs = boost::filesystem;
    auto p = fs::path (mp->filename).parent_path().string();
    if (!p.empty() && !fs::is_directory(p))
    {
      LOG_LINE << "creating directories: " << p;
      fs::create_directories(p);
    }

    mp->writer->Update();
  }
}
}
