#include "private/muk.pch"
#include "private/program_functions.h"
#include "private/program_options.h"

#include "MmwhsExperiment.h"
#include "SegThorExperiment.h"

#include "MukPathPlanning/PluginManagerPP.h"
#include "MukPdmWrapper/PluginManagerPdmWrapper.h"
#include "MukConvexOptimization/PluginManagerConvexOptimization.h"

#include <ctime>
#include <iomanip>

namespace
{
  using namespace gris;
  using namespace gris::muk;
  void createPatientDataFile();
}

/** /brief reads an xml input file to get all input parameters
  
  proceeds to ...
*/
int main(int argc, char ** argv)
{
  ProgramInput programInput;
  try
  {
    // set up logger
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "log_";
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    oss << ".txt";
    GetGrisLogger().setFilename(oss.str());
    GetGrisLogger().setFileLogging(true);

    PluginManagerPP::initialize();
    PluginManagerPdmWrapper::initialize();
    PluginManagerConvexOptimization::initialize();

    // read in configuration file
    if (!process_command_line(argc, argv, programInput))
    {
      LOG_LINE << "Error in process_command_line";
      return 0;
    }

    const auto root = programInput.root().getChild("Conf_MICCAI2019_OR20");
    const auto exps = root.getChild("Experiments");

    auto mmwhs = std::make_unique<MmwhsExperiment>();
    {
      auto mmwhsRoot = exps.getChild("MMWHS");
      mmwhs->initialize(mmwhsRoot);
      mmwhs->print();
      mmwhs->run();
      mmwhs->evaluate();
      mmwhs->finalize(mmwhsRoot);
    }
    auto segthor = std::make_unique<SegThorExperiment>();
    {

      auto segthorRoot = exps.getChild("SegThor");
      segthor->initialize(segthorRoot);
      segthor->print();
      segthor->run();
      segthor->evaluate();
      segthor->finalize(segthorRoot);
    }

/*
    if (programInput.diceData.trainModels)
      trainModels(programInput);
    if (programInput.diceData.run)
      runDiceExperiment(programInput);
    if (programInput.diceData.evaluate)
      evaluateDiceExperiment(programInput);*/
    
    /*if ( (programInput.cochleaData.run    && programInput.cochleaData.planningConfig.forceObstacleExtraction)
      || (programInput.vestibularData.run && programInput.vestibularData.planningConfig.forceObstacleExtraction))
    {
      extractObstacles(programInput);
    }*/
/*
    if (programInput.cochleaData.run)
      runCochleaExperiment(programInput);
    if (programInput.cochleaData.evaluate)
      evaluateCochleaExperiment(programInput);

    if (programInput.vestibularData.run)
      runVestibularExperiment(programInput);
    if (programInput.vestibularData.evaluate)
      evaluateVestibularExperiment(programInput);

    if (programInput.validatePasmData.run)
      runValidatePASMExperiment(programInput);
    if (programInput.validatePasmData.evaluate)
      evaluateValidatePASMExperiment(programInput);*/
    gris::XmlDocument::save(programInput.configFile, *programInput.pConfigDoc);
  }
  catch (std::exception& e)
  {
    LOG_LINE << e.what();
    XmlDocument::save(programInput.configFile, *programInput.pConfigDoc);
    return 1;
  }
}