#pragma once
#include "MukAlgorithms/AlgorithmWrapper.h"

#include <string>
#include <vector>

namespace gris
{
  namespace muk
  {
    enum EnExperimentType
    {
      enDice,
      enMonteCarlo,
      enCochlear,
      enVestibular,
      enValidatePASM,
      enPreprocessingAlgo,
      enSizeScenarioTypes
    };

    static const char* ScenarioNames[enSizeScenarioTypes] = 
    {
      "DiceExperiment",
      "MonteCarloExperiment",
      "CochleaExperiment",
      "VestibularSchwannomaExperiment",
      "DiceExperiment",
      "PreprocessingAlgorithm"
    };

    static const std::vector<std::string> organs = 
    { 
      "ChordaTympani",
      "Cochlea",
      "FacialNerve",
      "ExternalAuditoryCanal",
      "InternalCarotidArtery",
      "InternalAuditoryCanal",
      "JugularVein",
      "Ossicles",
      "SemicircularCanals"
    };
    
    static const std::vector<int> labels = 
    { 
      5,4,3,10,1,8,2,6,7
    };

    /**
    */
    struct PatientData
    {
      std::string id;
      std::string sceneFile;
      std::string ctFile;
      std::string gtFile;
    };

    /**
    */
    struct ExperimentInput
    {
      // parameters stable across experiments
      static std::string S_AppConfigFile; // config file with the properties below
      static std::string S_PatientDataFile;
      static std::string S_PlannerFile;

      // individual data
      std::vector<PatientData> patientData;
      std::string outputDir;
      std::string localBasePath;  // base directory for scene file data
      std::string dataSetRootDir; // base directory for ct/gt data
      std::string subResultDir;
      mutable std::string lastEvalDate; // this will be overwritten in the config file if an experiment or algorithm has been run
      
      // run experiment?
      bool run;
      // evaluate experiment ?
      bool evaluate;

      // skip some data sets?
      std::vector<std::string> skipPatients;
      std::vector<unsigned int> skipPatientsUnequal;
      int skipPatientsLarger;
      int skipPatientsSmaller;
    };

    /**
    */
    struct AlgInfo : public ExperimentInput
    {
      AlgInfo(const ExperimentInput& o)
        : ExperimentInput(o)
      {
      }

      int priority;
      std::string alias;
      std::string filename;
      std::string parentAlgorithm;
      mutable std::string parentAlgorithmEvalDate;
      std::vector<unsigned int> ids;
      std::vector<unsigned int> outIds;
      std::vector<EnDataType>   outTypes;
    };

    /**
    */
    struct DiceExperimentInput : public ExperimentInput
    {
      DiceExperimentInput()
        : mManualFirstDataSets(*this)
        , mManualSecondDataSets(*this)
        , mAutomatedInitFirst(*this)
        , mAutomatedPasmFirst(*this)
        , mAutomatedInitSecond(*this)
        , mAutomatedPasmSecond(*this)
      {
        subResultDir = ScenarioNames[enDice];
      }

      bool trainModels;
      bool createMeshes;
      bool overwriteMeshes;
      bool createCorrespondences;
      bool createSSM;
      bool createASM;

      AlgInfo mManualFirstDataSets;
      AlgInfo mManualSecondDataSets;

      AlgInfo mAutomatedInitFirst;
      AlgInfo mAutomatedPasmFirst;
      AlgInfo mAutomatedInitSecond;
      AlgInfo mAutomatedPasmSecond;
    };

    /**
    */
    struct MonteCarloExperimentInput : public ExperimentInput
    {
      MonteCarloExperimentInput() 
      {
        subResultDir = ScenarioNames[enMonteCarlo];
      }
    };
    
    /**
    */
    struct PlanningConfig
    {
      std::vector<std::string> accessCanals;
      std::string planner;
      std::string interpolator;
      std::string segmentationDirAuto;
      std::string segmentationDirManual;
      bool forceObstacleExtraction = false;
      double planningTime = 1.0;

      using KeyValuePair = std::pair<std::string, std::string>;
      std::vector<KeyValuePair> plannerParams;
      std::vector<KeyValuePair> interpolatorParams;

      void print() const;
    };

    /**
    */
    struct CochleaExperimentInput : public ExperimentInput
    {
      CochleaExperimentInput() 
      {
        subResultDir = ScenarioNames[enCochlear];
      }

      PlanningConfig planningConfig;
    };

    /**
    */
    struct VestibularExperimentInput : public ExperimentInput
    {
      VestibularExperimentInput() 
      {
        subResultDir = ScenarioNames[enVestibular];
      }

      PlanningConfig planningConfig;
    };

    /**
    */
    struct ValidatePASMExperimentInput : public ExperimentInput
    {
      ValidatePASMExperimentInput() 
      {
        subResultDir = ScenarioNames[enValidatePASM];
      }
    };
    
    /**
    */
    struct ProgramInput
    {
      std::vector<AlgInfo> preprocessingAlgs;
      
      DiceExperimentInput  diceData;
      MonteCarloExperimentInput    monteData;
      CochleaExperimentInput  cochleaData;
      VestibularExperimentInput vestibularData;
      ValidatePASMExperimentInput validatePasmData;
    };

    bool process_command_line(int argc, char ** argv, ProgramInput& programInput);
    void readConfigFile(ProgramInput& programInput);
  }
}
