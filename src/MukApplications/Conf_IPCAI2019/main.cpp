#include "private/muk.pch"
#include "private/program_functions.h"
#include "private/program_options.h"

#include "CochleaExperiment.h"
#include "VestibularExperiment.h"

#include "MukPathPlanning/PluginManagerPP.h"

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
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "log_";
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    oss << ".txt";
    GetGrisLogger().setFilename(oss.str());

    GetGrisLogger().setFileLogging(true);
    if (!process_command_line(argc, argv, programInput))
    {
      LOG_LINE << "Error in process_command_line";
      return 0;
    }

    PluginManagerPP::initialize();

    readConfigFile(programInput);

    for (const auto& alg : programInput.preprocessingAlgs)
    {
      if (alg.run)
        preprocess(programInput, alg);
    }
        
    if (programInput.cochleaData.run)
      runCochleaExperiment(programInput);
    if (programInput.cochleaData.evaluate)
      evaluateCochleaExperiment(programInput);

    if (programInput.vestibularData.run)
      runVestibularExperiment(programInput);
    if (programInput.vestibularData.evaluate)
      evaluateVestibularExperiment(programInput);
  }
  catch (std::exception& e)
  {
    LOG_LINE << "Error occured: " << e.what();
    return 0;
  }
  return EXIT_SUCCESS;
}