#include "private/muk.pch"
#include "program_options.h"

#include "SegThorExperiment.h"

#include "MukPathPlanning/PluginManagerPP.h"
#include "MukConvexOptimization/PluginManagerConvexOptimization.h"

#include <memory>
#include <iomanip>

namespace
{
  using namespace gris;
  using namespace gris::muk;
}

/** /brief reads an xml input file to get all input parameters
  
  proceeds to ...
*/
int main(int argc, char ** argv)
{
  ProgramInput programInput;
  try
  {
    // create log
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "log_";
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    oss << ".txt";
    GetGrisLogger().setFilename(oss.str());
    GetGrisLogger().setFileLogging(true);

    // check input
    if ( ! process_command_line(argc, argv, programInput))
      return 0;

    // prepare plugins
    PluginManagerPP::initialize();
    PluginManagerConvexOptimization::initialize();

    // run experiments
    auto pSegThorExp      = std::make_unique<SegThorExperiment>();
    {
      const auto root = programInput.root().getChild("Conf_IROS2019");
      const auto exps = root.getChild("Experiments");
      
      auto segThorRoot = exps.getChild("SegThorExperiment");
      pSegThorExp->initialize(segThorRoot);
      pSegThorExp->print();
      pSegThorExp->run();
      pSegThorExp->evaluate();
      pSegThorExp->finalize(segThorRoot);

      XmlDocument::save(programInput.configFile, *programInput.pConfigDoc);
    }
  }
  catch (std::exception& e)
  {
    LOG_LINE << e.what();
    XmlDocument::save(programInput.configFile, *programInput.pConfigDoc);
    return 1;
  }
  return 0;
}