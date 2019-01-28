#include "private/muk.pch"
#include "LocalEnvironment.h"

#include "MukCommon/MukException.h"

#include "gstd/XmlDocument.h"
#include "gstd/XmlNode.h"

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <list>
#include <iostream>

namespace gris
{
namespace muk
{
  /**
  */
  LocalEnvironment::LocalEnvironment()
    : mRawDataPath("../")
    , mFileName("../resources/local.mukno")
    , mMaxSize(25)
    , mBinaryDirectory("")
  {
    declareProperty<std::string>("RawDataPath",
      std::bind(&LocalEnvironment::setRawDataPath, this, std::placeholders::_1),
      std::bind(&LocalEnvironment::getRawDataPath, this));
  }

  void LocalEnvironment::setMaxSize(size_t size)
  {
    mLastSceneFiles.resize(size);
  }

  /**
  */
  void LocalEnvironment::setAsLastFilename(const std::string& filename)
  {
    mLastSceneFiles.remove(filename);
    mLastSceneFiles.push_front(filename);
    if (mLastSceneFiles.size() > mMaxSize)
      mLastSceneFiles.resize(mMaxSize);
  }

  /**
  */
  void LocalEnvironment::save() const
  {
    namespace fs = boost::filesystem;
    fs::path fn (mFileName);
    auto pDoc = gris::XmlDocument::create("Mukno");
    auto root = pDoc->getRoot();
    auto node = root.addChild("LocalEnvironment");
    node.addChild("RawDataPath").setValue(mRawDataPath.c_str());
    node.addChild("Filename").setValue(mFileName.c_str());
    node = node.addChild("LastScenes");
    for (const auto& file : mLastSceneFiles)
    {
      node.addChild("Scene").setValue(file.c_str());
    }
    gris::XmlDocument::save(mFileName.c_str(), *pDoc);
  }

  /**
  */
  void LocalEnvironment::load()
  {
    namespace fs = boost::filesystem;
    fs::path fn(mFileName);
    if (!mBinaryDirectory.empty() && fn.is_absolute())
      fn = fs::path(mBinaryDirectory) / fn;
    // if the file does not exist, create it
    if ( !fs::exists(fn))
    {
      fs::create_directories(fn.parent_path());
      save();
    }
    auto pDoc = gris::XmlDocument::read(mFileName.c_str());
    auto root = pDoc->getRoot();
    auto node = root.getChild("LocalEnvironment");
    mRawDataPath = node.getChild("RawDataPath").getValue();
    mFileName    = node.getChild("Filename").getValue();
    auto sceneNodes = node.getChild("LastScenes").getChildren();
    for (const auto& node : sceneNodes)
    {
      mLastSceneFiles.push_back(node.getValue());
    }
  }
}
}