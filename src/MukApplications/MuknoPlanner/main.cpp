#include "private/muk.pch"
#include <private/RegisterQtMetaTypes.hxx>

#include "MuknoPlanner.h"
#include "SafeApplication.h"

#include "MukCommon/version.h"
#include "MukNavigation/PluginManagerNavi.h"
#include "MukPathPlanning/PluginManagerPP.h"
// The required Versions are provided in private/requiredLibraryVersions.hxx.in.
// CMAKE will configure the local required Version variables into a file in the binary directory.
#include "private/requiredLibraryVersions.hxx"

#include <boost/format.hpp>

#include <windows.h>

using namespace gris::muk;

#define DEVELOP

void checkLibraryVersions();

int main(int argc, char ** argv)
{
  //gris::GetGrisLogger().setCoutLogging(true);
  //gris::GetGrisLogger().setFileLogging(true);
  registerMetaTypes();

  try
  {
    // this has to be done before the gui, elsewise it may crash
    checkLibraryVersions();

    gris::muk::PluginManagerPP::initialize();
    gris::muk::PluginManagerNavi::initialize();

#ifndef DEVELOP
    ShowWindow( GetConsoleWindow(), SW_HIDE );
#endif
    SafeApplication app(argc, argv);
    
    std::string sceneFile;
    std::string ctFile;
    auto pMainApp = MuknoPlanner::create(argv[0]);
    for (int i(1); i<argc; ++i)
    {
      pMainApp->handleFileInput(argv[i]);
    }
    pMainApp->showInStartUpMode();
    app.exec();
    return 0;
  }
  catch (std::exception& e)
  {
    LOG_LINE << "Error: " << e.what();
  #ifdef DEVELOP
    // keep the console window open, on a fatal crash in "DEVELOP" MODE
    std::cout << "FATAL CRASH: Press [Enter] to exit" << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  #endif
    return 1;
  }
}

#ifdef max
#undef max
#endif

namespace std 
{
  ostream& operator<<(ostream& os, const int versions[4])
  {
    os << versions[0] << "." << versions[1] << "." << versions[2] << "." << versions[3];
    return os;
  }
}

void checkLibraryVersions()
{
  using namespace gris::muk;

  bool all_dlls_good = true;
  bool any_mismatch  = false;
  for (auto& pair : libraries)
  {
    const auto& libName    = pair.first;
    const auto& libVersion = pair.second;
    DWORD major, minor, patch, tweak;
    gris::muk::getMuknoLibraryVersion(libName.c_str(), major, minor, patch, tweak);
    const std::array<unsigned int,4> versions { major, minor, patch, tweak };
    std::string prefix = "WARNING: ";
    if ( major != libVersion[0] 
      || minor != libVersion[1]) 
    {
      all_dlls_good = false;
      prefix = "ERROR: ";
    }
    const bool mismatch = ! std::equal(std::begin(versions), std::end(versions), std::begin(libVersion));
    std::string msg;
    any_mismatch |= mismatch;
    if (mismatch)
    {
      msg = (boost::format("The required library %s was found with the wrong version.\n"
        "    Required version number: %d.%d.%d.%d.\n"
        "    Present version number:  %d.%d.%d.%d")
        % libName % libVersion[0] % libVersion[1] % libVersion[2] % libVersion[3] % major % minor % patch % tweak).str();
      LOG_LINE << prefix << "Version mismatch detected: ";
      LOG_LINE << msg;
    }
  }
  if (any_mismatch) {
    LOG_LINE << "Press a [Enter] to " << (all_dlls_good ? "continue" : "exit");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  if (!all_dlls_good)
    throw std::runtime_error("library version mismatch!");
}