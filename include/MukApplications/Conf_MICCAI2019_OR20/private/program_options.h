#pragma once
#include <gstd/XmlDocument.h>
#include <gstd/XmlNode.h>

#include <string>
#include <vector>

namespace gris
{
  namespace muk
  {
    /** \brief
    */
    static const std::vector<std::pair<std::string, int>> SegThorLabels = 
    { 
      {"Esophagus",1},
      {"Heart",2},
      {"Trachea",3},
      {"Aorta",4},
    };

    /** \brief
    */
    struct LabelImageToMeshConfig
    {
      std::string fileTemplate;

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
      std::string name; /// name of the experiment, can be used in various ways
      // general stuff
      std::string outputRootDir;
      std::string dataSetRootDir; // base directory for ct/gt data
      //skip some data sets?
      std::vector<std::string> skipPatients;
      std::vector<unsigned int> skipPatientsUnequal;
      int skipPatientsLarger;
      int skipPatientsSmaller;
      
      // other stuff
      std::string plannerFile;

      // individual data
      std::vector<PatientData> patientData;
      //std::string subResultDir;
      mutable std::string lastEvalDate; // this will be overwritten in the config file if an experiment or algorithm has been run
      
      // run experiment?
      bool run;
      // evaluate experiment ?
      bool evaluate;
    };

    /**
    */
    struct ProgramInput
    { 
      XmlNode root() const { return pConfigDoc->getRoot(); }

      std::string configFile = "../resources/MukApplications/Conf_MICCAI2019_OR20/Conf_MICCAI2019_OR20.mukcfg";
      std::unique_ptr<XmlDocument> pConfigDoc;
    };

    bool process_command_line(int argc, char ** argv, ProgramInput& programInput);
  }
}
