#include "private/muk.pch"
#include "program_options.h"

#include "MukCommon/LocaleBool.h"
#include "MukCommon/MukException.h"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace
{
}

namespace gris
{
namespace muk
{
  /**
  */
  XmlNode ProgramInput::root()
  {
    return pConfigDoc->getRoot();
  }

  /** \brief Implements a help dialog for console applications and reads in minimal input.

    The function throws if the input can't be parsed.

    \return 0, if option -help or -h was requested, else 1
  */
  bool process_command_line(int argc, char ** argv, ProgramInput& programInput)
  {
    namespace po = boost::program_options;
    po::options_description desc("Options");
    std::ostringstream sshelp;
    sshelp << "This application runs the evalautions for the MICCAI 2019 paper 'Optimizing Clearance of Bézier Spline Trajectories for Minimally-Invasive Surgery'\n";
    desc.add_options()
      ("help,h", sshelp.str().c_str())
      (",f", po::value<std::string>(&programInput.configFile), "the application's config file and only input");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm); // can throw
    if (vm.count("help"))
    {
      LOG_LINE << desc;
      return false;
    }
    
    namespace fs = boost::filesystem;
    if ( ! fs::is_regular_file(programInput.configFile))
    {
      throw MUK_EXCEPTION("config file does not exist", programInput.configFile.c_str());
    }
    // validate
    LOG_LINE << " ------------------------------------------------------------------------";
    LOG_LINE << "running application based on config file " << programInput.configFile.c_str();
    LOG_LINE << " ------------------------------------------------------------------------";
    programInput.pConfigDoc = std::move(XmlDocument::read(programInput.configFile.c_str()));

    return true;
  }
}
}