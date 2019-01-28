#include "private/muk.pch"
#include "MukObstacle.h"
#include "mukIO.h"
#include "MukException.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include <boost/filesystem.hpp>

namespace
{
  namespace fs = boost::filesystem;
}

namespace gris
{
  namespace muk
  {
    /**
    */
    MukObstacle::MukObstacle()
      : mActive(true)
      , mData(make_vtk<vtkPolyData>())
    {
      declareProperty<std::string>("Name",
        std::bind(&MukObstacle::setName, this, std::placeholders::_1), 
        std::bind(&MukObstacle::getName, this));
      declareProperty<std::string>("File",
        std::bind(&MukObstacle::setFileName, this, std::placeholders::_1), 
        std::bind(&MukObstacle::getFileName, this));
      declareProperty<bool>("Active",
        std::bind(&MukObstacle::setActive, this, std::placeholders::_1), 
        std::bind(&MukObstacle::getActive, this));
    }

    /**
    */
    MukObstacle::~MukObstacle()
    {
    }

    /**
    */
    void MukObstacle::swap(MukObstacle& o)
    {
      mName.swap(o.mName);
      mFileName.swap(o.mFileName);
      auto tmp = mData;
      mData = o.mData;
      o.mData = tmp;
    }

    /**
    */
    void MukObstacle::load()
    {
      fs::path p (mFileName);
      if (!fs::is_regular_file(p))
      {
        throw MUK_EXCEPTION("Cannot read file", p.string().c_str());
      }
      if (p.extension().string() == ".vtk")
      {
        mData = loadVtkFile(mFileName);
      }
      else if (p.extension().string() == ".obj")
      {
        mData = loadObjFile(mFileName);
      }
      else if (p.extension().string() == ".stl")
      {
        mData = loadStlFile(mFileName);
      }
      else if (p.extension().string() == ".mhd")
      {
        mData = loadMhdFile(mFileName);
      }
      else
      {
        throw MUK_EXCEPTION("Cannot read file. Unsupported file type!", p.string().c_str());
      }
    }

    /**
    */
    void MukObstacle::save() const
    {
      saveVtkPolyData(mData, mFileName);
    }
  }
}