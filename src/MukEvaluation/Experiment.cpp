#include "private/muk.pch"
#include "Experiment.h"

#include "MukCommon/LocaleBool.h"
#include "MukCommon/muk_common.h"
#include "MukCommon/muk_dynamic_property_tools.h"
#include "MukCommon/gris_property_streams.h"

#include <boost/filesystem.hpp>

#include <iomanip>

namespace gris
{
namespace muk
{
  /**
  */
  Experiment::Experiment()
    : mRun(false)
    , mEvaluate(false)
    , mSkipPatientsHigher(-1)
    , mSkipPatientsLesser(-1)
  {
    mProps.declareProperty<bool>("Compute", MUK_D_SET(bool, mRun), MUK_D_GET(mRun));
    mProps.declareProperty<bool>("Evaluate", MUK_D_SET(bool, mEvaluate), MUK_D_GET(mEvaluate));
    mProps.declareProperty<std::vector<unsigned int>>("SkipPatientsEqual", MUK_D_C_SET(std::vector<unsigned int>, mSkipPatientsEqual), MUK_D_C_GET(std::vector<unsigned int>, mSkipPatientsEqual));
    mProps.declareProperty<std::vector<unsigned int>>("SkipPatientsUnequal", MUK_D_C_SET(std::vector<unsigned int>, mSkipPatientsUnequal), MUK_D_C_GET(std::vector<unsigned int>, mSkipPatientsUnequal));
    mProps.declareProperty<int>("SkipPatientsHigher", MUK_D_SET(int, mSkipPatientsHigher), MUK_D_GET(mSkipPatientsHigher));
    mProps.declareProperty<int>("SkipPatientsLesser", MUK_D_SET(int, mSkipPatientsLesser), MUK_D_GET(mSkipPatientsLesser));
    mProps.declareProperty<std::string>("LastEvaluationDate", MUK_D_C_SET(std::string, mLastEvalDate), MUK_D_C_GET(std::string, mLastEvalDate));
    mProps.declareProperty<std::string>("ResourceDir", MUK_D_C_SET(std::string, mResourceDir), MUK_D_C_GET(std::string, mResourceDir));
    mProps.declareProperty<std::string>("OutputRootDir", MUK_D_C_SET(std::string, mOutputRootDir), MUK_D_C_GET(std::string, mOutputRootDir));
    mProps.declareProperty<std::string>("Subdir", MUK_D_C_SET(std::string, mSubDir), MUK_D_C_GET(std::string, mSubDir));
  }

  /** \brief reads existing parameters from an XmlNode
  
    \param[in] node the root node of this experiment
  */
  void Experiment::initialize(const XmlNode& node)
  {
    auto read = [&] (const std::string& key, gstd::DynamicProperty& prop)
      {
        if (node.hasChild(key))
        {
          prop.setProperty(key, node.getChild(key).getValue());
        }
    };
    std::vector<std::string> names;
    mProps.getPropertyNames(names);
    for(const auto& name: names)
      read(name, mProps);
    // overwrite lastEvalDate if necessary
    if (mRun)
    {
      // created date;
      auto t  = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::ostringstream oss;
      oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
      mLastEvalDate = oss.str();
    }
  }

  /** \brief writes its parameters into an XmlNode

    \param[in] node the root node of this experiment
  */
  void Experiment::finalize(XmlNode& node)
  {
    auto write = [&] (const std::string& key, gstd::DynamicProperty& prop)
    {
      XmlNode res;
      if (!node.hasChild(key))
        res = node.addChild(key);
      else
        res = node.getChild(key);
      std::string val;
      prop.getProperty(key, val);
      res.setValue( val );
    };
    std::vector<std::string> names;
    mProps.getPropertyNames(names);
    for(const auto& name: names)
      write(name, mProps);
    // copy log file
    // save log file for debugging
    namespace fs = boost::filesystem;
    const auto dir = fs::path(mOutputRootDir) / mSubDir / mLastEvalDate;
    if ( ! fs::is_directory(dir))
    {
      fs::create_directories(dir);
    }
    fs::copy_file( gris::GetGrisLogger().getFilename(), dir / "log_run.txt", fs::copy_option::overwrite_if_exists);
  }

  /**
  */
  void Experiment::print()
  {
    LOG_LINE << "====== Experiment " << mName << "==========";
    std::vector<std::string> names;
    mProps.getPropertyNames(names);
    for(const auto& name: names)
    {
      std::string val;
      mProps.getProperty(name, val);
      LOG_LINE << "   " << name << ": " << val;
    }
  }

  /**
  */
  bool Experiment::skipId(unsigned int id)
  {
    if (std::any_of(mSkipPatientsEqual.begin(), mSkipPatientsEqual.end(), [&] (auto i) { return i == id; }))
      return true;
    if (! mSkipPatientsUnequal.empty() &&  std::none_of(mSkipPatientsUnequal.begin(), mSkipPatientsUnequal.end(), [&] (auto i) { return i == id; }))
      return true;
    if (mSkipPatientsHigher >= 0 && (int)id >= mSkipPatientsHigher)
      return true;
    if (mSkipPatientsLesser >= 0 && (int)id <= mSkipPatientsLesser)
      return true;
    return false;
  }
}
}