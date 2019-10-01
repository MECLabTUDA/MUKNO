#include "private/muk.pch"
#include "private/program_options.h"

#include "MukCommon/LocaleBool.h"
#include "MukCommon/MukException.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace
{
  namespace fs = boost::filesystem;
  using namespace gris;
  using namespace gris::muk;
}

namespace gris
{
namespace muk
{
    /**
    */
    bool process_command_line(int argc, char ** argv, ProgramInput& programInput)
    {
      namespace po = boost::program_options;
      po::options_description desc("Options");
      std::ostringstream sshelp;
      sshelp << "This application runs the evalautions for the MICCAI 2019 OR 2.0 paper\n";
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
      programInput.pConfigDoc = std::move(XmlDocument::read(programInput.configFile.c_str()));
      return true;
    }

//    //  static_cast<ExperimentInput&>(input.diceData)   = shared;
//    //  static_cast<ExperimentInput&>(input.cochleaData)   = shared;
//    //  static_cast<ExperimentInput&>(input.vestibularData)  = shared;
//    //  static_cast<ExperimentInput&>(input.validatePasmData) = shared;
//    //   
//    //  auto algNodes = rootNode.getChild("Preprocessing").getChildren();
//    //  for (const auto& node : algNodes)
//    //  {
//    //    AlgInfo info = shared;
//    //    loadAlgo(info, node, true);
//    //    input.preprocessingAlgs.push_back(info);
//    //  }
//
//    //  const auto expNode = rootNode.getChild("Experiments");
//    //  baseNode = expNode.getChild(ScenarioNames[enDice]);
//    //  {
//    //    auto& data = input.diceData;
//    //    data.run          = boost::lexical_cast<LocaleBool>(baseNode.getChild("Compute").getValue());
//    //    data.evaluate     = boost::lexical_cast<LocaleBool>(baseNode.getChild("Evaluate").getValue());
//    //    auto trainingNode = baseNode.getChild("ModelTraining");
//    //    {
//    //      data.trainModels  = boost::lexical_cast<LocaleBool>(trainingNode.getChild("TrainModels").getValue());
//    //      data.createMeshes           = boost::lexical_cast<LocaleBool>(trainingNode.getChild("SurfaceExtraction").getValue());
//    //      data.overwriteMeshes        = boost::lexical_cast<LocaleBool>(trainingNode.getChild("OverwriteExistingSurfaces").getValue());
//    //      data.createCorrespondences  = boost::lexical_cast<LocaleBool>(trainingNode.getChild("Correspondence").getValue());
//    //      data.createSSM              = boost::lexical_cast<LocaleBool>(trainingNode.getChild("SSM").getValue());
//    //      data.createASM              = boost::lexical_cast<LocaleBool>(trainingNode.getChild("ASM").getValue());
//    //    }
//    //    data.lastEvalDate = baseNode.getChild("LastEvaluationDate").getValue();
//    //    auto algosNode    = baseNode.getChild("Algorithms").getChildren();
//    //    {
//    //      // ---------------------------------------------------------------------------------------------------------------------------
//    //      // manual first data set
//    //      auto algoname = "ManualPASM_PASM1";
//    //      auto iter = findDiceAlgo(algosNode, algoname);
//    //      data.mManualFirstDataSets = shared;
//    //      loadAlgo(data.mManualFirstDataSets, *iter, false);
//    //      {
//    //        // make sure, the algo is evaluated only for the second data set
//    //        auto& v = data.mManualFirstDataSets.patientData;
//    //        v.erase(std::remove_if(v.begin(), v.end(), [&] (const auto& p) { return std::stoi(p.id.substr(1,2)) <= 15; }), v.end());
//    //      }
//    //      // ---------------------------------------------------------------------------------------------------------------------------
//    //      // manual second data set
//    //      algoname = "ManualPASM_PASM2";
//    //      iter = findDiceAlgo(algosNode, algoname);
//    //      data.mManualSecondDataSets = shared;
//    //      loadAlgo(data.mManualSecondDataSets, *iter, false);
//    //      {
//    //        // make sure, the algo is evaluated only for the first data set
//    //        auto& v = data.mManualSecondDataSets.patientData;
//    //        v.erase(std::remove_if(v.begin(), v.end(), [&] (const auto& p) { return std::stoi(p.id.substr(1,2)) > 15; }), v.end());
//    //      }
//    //      // ---------------------------------------------------------------------------------------------------------------------------
//    //      // automatic init algo first data set
//    //      algoname = "UnetToPASMInit_PASM1";
//    //      iter = findDiceAlgo(algosNode, algoname);
//    //      data.mAutomatedInitFirst = shared;
//    //      loadAlgo(data.mAutomatedInitFirst, *iter, false);
//    //      {
//    //        // make sure, the algo is evaluated only for the second data set
//    //        auto& v = data.mAutomatedInitFirst.patientData;
//    //        v.erase(std::remove_if(v.begin(), v.end(), [&] (const auto& p) { return std::stoi(p.id.substr(1,2)) <= 15; }), v.end());
//    //      }
//    //      // ---------------------------------------------------------------------------------------------------------------------------
//    //      // automatic pasm algo first data set
//    //      algoname = "AutomatedPASM_PASM1";
//    //      iter = findDiceAlgo(algosNode, algoname);
//    //      data.mAutomatedPasmFirst = shared;
//    //      loadAlgo(data.mAutomatedPasmFirst, *iter, false);
//    //      {
//    //        // make sure, the algo is evaluated only for the first data set
//    //        auto& v = data.mAutomatedPasmFirst.patientData;
//    //        v.erase(std::remove_if(v.begin(), v.end(), [&] (const auto& p) { return std::stoi(p.id.substr(1,2)) <= 15; }), v.end());
//    //      }
//    //      iter = findDiceAlgo(algosNode, data.mAutomatedPasmFirst.parentAlgorithm);
//    //      data.mAutomatedPasmFirst.parentAlgorithmEvalDate = iter->getChild("LastEvaluationDate").getValue();
//    //      // ---------------------------------------------------------------------------------------------------------------------------
//    //      // automatic init algo second data set
//    //      algoname = "UnetToPASMInit_PASM2";
//    //      iter = findDiceAlgo(algosNode, algoname);
//    //      data.mAutomatedInitSecond = shared;
//    //      loadAlgo(data.mAutomatedInitSecond, *iter, false);
//    //      {
//    //        // make sure, the algo is evaluated only for the second data set
//    //        auto& v = data.mAutomatedInitSecond.patientData;
//    //        v.erase(std::remove_if(v.begin(), v.end(), [&] (const auto& p) { return std::stoi(p.id.substr(1,2)) > 15; }), v.end());
//    //      }
//    //      // ---------------------------------------------------------------------------------------------------------------------------
//    //      // automatic pasm algo second data set
//    //      algoname = "AutomatedPASM_PASM2";
//    //      iter = findDiceAlgo(algosNode, algoname);
//    //      data.mAutomatedPasmSecond = shared;
//    //      loadAlgo(data.mAutomatedPasmSecond, *iter, false);
//    //      {
//    //        // make sure, the algo is evaluated only for the first data set
//    //        auto& v = data.mAutomatedPasmSecond.patientData;
//    //        v.erase(std::remove_if(v.begin(), v.end(), [&] (const auto& p) { return std::stoi(p.id.substr(1,2)) > 15; }), v.end());
//    //      }
//    //      iter = findDiceAlgo(algosNode, data.mAutomatedPasmSecond.parentAlgorithm);
//    //      data.mAutomatedPasmSecond.parentAlgorithmEvalDate = iter->getChild("LastEvaluationDate").getValue();
//    //    }
//    //  }
//    //  // old stuff when deep leraning and registration was planned for MICCAI, now out of date
//    //  baseNode = expNode.getChild(ScenarioNames[enCochlear]);
//    //  {
//    //    auto& data = input.cochleaData;
//    //    data.run          = boost::lexical_cast<LocaleBool>(baseNode.getChild("Compute").getValue());
//    //    data.evaluate     = boost::lexical_cast<LocaleBool>(baseNode.getChild("Evaluate").getValue());
//    //    data.lastEvalDate = baseNode.getChild("LastEvaluationDate").getValue();
//
//    //    const auto ndPlanning = baseNode.getChild("Planning");
//    //    auto& config          = data.planningConfig;
//    //    auto dir = std::string(ndPlanning.getChild("SegmentationDirectoryAuto").getValue());
//    //    if ( ! dir.empty())
//    //    {
//    //      config.segmentationDirAuto = dir;
//    //    }
//    //    else
//    //    {
//    //      const auto dir1 = fs::path(data.outputRootDir) / "AutomatedPASM_PASM2";
//    //      if (fs::is_directory(dir1))
//    //      {
//    //        for (auto iter = fs::directory_iterator(dir1); iter != fs::directory_iterator(); ++iter)
//    //        {
//    //          if ( ! fs::is_directory(iter->path()))
//    //            continue;
//    //          config.segmentationDirAuto = iter->path().string(); // take the latest
//    //        }
//    //      }
//    //    }
//    //    dir = std::string(ndPlanning.getChild("SegmentationDirectoryManual").getValue());
//    //    if ( ! dir.empty())
//    //    {
//    //      config.segmentationDirManual = dir;
//    //    }
//    //    else
//    //    {
//    //      const auto dir1 = fs::path(data.outputRootDir) / "ManualPASM_PASM2";
//    //      if (fs::is_directory(dir1))
//    //      {
//    //        for (auto iter = fs::directory_iterator(dir1); iter != fs::directory_iterator(); ++iter)
//    //        {
//    //          if ( ! fs::is_directory(iter->path()))
//    //            continue;
//    //          config.segmentationDirManual = iter->path().string(); // take the latest
//    //        }
//    //      }
//    //    }
//    //    config.planningTime   = std::atof(ndPlanning.getChild("PlanningTime").getValue());
//    //    config.planner        = ndPlanning.getChild("Planner").getValue();
//    //    config.interpolator   = ndPlanning.getChild("Interpolator").getValue();
//    //    for (const auto& nd : ndPlanning.getChild("PlannerParameters").getChildren())
//    //      config.plannerParams.push_back(std::make_pair<std::string, std::string>(nd.getName(), nd.getValue()));
//    //    for (const auto& nd : ndPlanning.getChild("InterpolatorParameters").getChildren())
//    //      config.interpolatorParams.push_back(std::make_pair<std::string, std::string>(nd.getName(), nd.getValue()));
//    //    for (const auto& nd : ndPlanning.getChild("AccessCanals").getChildren())
//    //    {
//    //      config.accessCanals.push_back(nd.getValue());
//    //    }
//    //    config.print();
//    //  }
//    //  baseNode = expNode.getChild(ScenarioNames[enVestibular]);
//    //  {
//    //    auto& data = input.vestibularData;
//    //    data.run          = boost::lexical_cast<LocaleBool>(baseNode.getChild("Compute").getValue());
//    //    data.evaluate     = boost::lexical_cast<LocaleBool>(baseNode.getChild("Evaluate").getValue());
//    //    data.lastEvalDate = baseNode.getChild("LastEvaluationDate").getValue();
//
//    //    const auto ndPlanning = baseNode.getChild("Planning");
//    //    auto& config          = data.planningConfig;
//    //    auto dir = std::string(ndPlanning.getChild("SegmentationDirectoryAuto").getValue());
//    //    if ( ! dir.empty())
//    //    {
//    //      config.segmentationDirAuto = dir;
//    //    }
//    //    else
//    //    {
//    //      const auto dir1 = fs::path(data.outputRootDir) / "AutomatedPASM_PASM2";
//    //      if (fs::is_directory(dir1))
//    //      {
//    //        for (auto iter = fs::directory_iterator(dir1); iter != fs::directory_iterator(); ++iter)
//    //        {
//    //          if ( ! fs::is_directory(iter->path()))
//    //            continue;
//    //          config.segmentationDirAuto = iter->path().string(); // take the latest
//    //        }
//    //      }
//    //    }
//    //    dir = std::string(ndPlanning.getChild("SegmentationDirectoryManual").getValue());
//    //    if ( ! dir.empty())
//    //    {
//    //      config.segmentationDirManual = dir;
//    //    }
//    //    else
//    //    {
//    //      const auto dir1 = fs::path(data.outputRootDir) / "ManualPASM_PASM2";
//    //      if (fs::is_directory(dir1))
//    //      {
//    //        for (auto iter = fs::directory_iterator(dir1); iter != fs::directory_iterator(); ++iter)
//    //        {
//    //          if ( ! fs::is_directory(iter->path()))
//    //            continue;
//    //          config.segmentationDirManual = iter->path().string(); // take the latest
//    //        }
//    //      }
//    //    }
//    //    config.planningTime   = std::atof(ndPlanning.getChild("PlanningTime").getValue());
//    //    config.planner        = ndPlanning.getChild("Planner").getValue();
//    //    config.interpolator   = ndPlanning.getChild("Interpolator").getValue();
//    //    for (const auto& nd : ndPlanning.getChild("PlannerParameters").getChildren())
//    //      config.plannerParams.push_back(std::make_pair<std::string, std::string>(nd.getName(), nd.getValue()));
//    //    for (const auto& nd : ndPlanning.getChild("InterpolatorParameters").getChildren())
//    //      config.interpolatorParams.push_back(std::make_pair<std::string, std::string>(nd.getName(), nd.getValue()));
//    //    for (const auto& nd : ndPlanning.getChild("AccessCanals").getChildren())
//    //    {
//    //      config.accessCanals.push_back(nd.getValue());
//    //    }
//    //    config.print();
//    //  }
//    //  baseNode = expNode.getChild(ScenarioNames[enValidatePASM]);
//    //  {
//    //    auto& data = input.validatePasmData;
//    //    data.run          = boost::lexical_cast<LocaleBool>(baseNode.getChild("Compute").getValue());
//    //    data.evaluate     = boost::lexical_cast<LocaleBool>(baseNode.getChild("Evaluate").getValue());
//    //    data.lastEvalDate = baseNode.getChild("LastEvaluationDate").getValue();
//    //  }
//    //  for (auto& info : input.preprocessingAlgs)
//    //  {
//    //    auto parentAlgo = info.parentAlgorithm;
//    //    if (parentAlgo.empty())
//    //      continue;
//    //    auto iter = std::find_if(input.preprocessingAlgs.begin(), input.preprocessingAlgs.end(), [&] (const auto& alg) { return alg.alias == parentAlgo; });
//    //    if (iter == input.preprocessingAlgs.end())
//    //      throw MUK_EXCEPTION("parent algorithm alias missing", parentAlgo.c_str());
//    //    info.parentAlgorithmEvalDate = iter->lastEvalDate;
//    //    LOG_LINE << "change " << info.alias << "'s parent data to " << info.parentAlgorithmEvalDate;
//    //  }
//    //}
//  }
//
//  /*void PlanningConfig::print() const
//  {
//    LOG << "Planning Config:\n"
//      << "  using " << planner << " and " << interpolator << " for\n";
//    for (const auto& c : accessCanals)
//      LOG << "     " << c << "\n";
//    LOG << "  in " << planningTime << "s\n";
//    LOG << "  using planning params\n";
//    for (const auto& pair : plannerParams)
//      LOG << "     " << pair.first << " : " << pair.second << "\n";
//    LOG << "  using interpolator params\n";
//    for (const auto& pair : interpolatorParams)
//      LOG << "     " << pair.first << " : " << pair.second << "\n";
//    LOG_LINE;
}
}

//namespace
//{
//  /** \brief loads ct-, ground-truth- and scene-files for patients.
//  */
//  std::vector<PatientData> loadPatientData(const std::string& file)
//  {
//    if (!fs::is_regular_file(file))
//      throw MUK_EXCEPTION("invalid patient data file", file.c_str());
//    auto pDoc = XmlDocument::read(file.c_str());
//    const auto rootNode = pDoc->getRoot();
//
//    auto imageRootDir = fs::path(rootNode.getChild("ImageRootDir").getValue());
//    auto sceneRootDir = fs::path(rootNode.getChild("SceneRootDir").getValue());
//    const auto nodes = rootNode.getChild("Patients").getChildren();
//
//    std::vector<PatientData> result;
//    for (const auto& node : nodes)
//    {
//      PatientData next;
//      next.id = node.getChild("ID").getValue();
//      next.ctFile = (imageRootDir / next.id / node.getChild("CT-Image").getValue()).generic_string();
//      next.gtFile = (imageRootDir / next.id / node.getChild("GT-Image").getValue()).generic_string();
//      next.sceneFile = (sceneRootDir / node.getChild("SceneFile").getValue()).generic_string();;
//      result.push_back(next);
//    }
//    return result;
//  }
//
//  ///** \brief loads algorithm parameters from a xml file
//  //*/
//  //void loadAlgo(AlgInfo& info, const gris::XmlNode& node, bool saveDate)
//  //{
//  //  info.run      = boost::lexical_cast<LocaleBool>(node.getChild("Compute").getValue());
//  //  info.priority = std::atoi(node.getChild("Priority").getValue());
//  //  info.alias    = node.getChild("Alias").getValue();
//  //  LOG_LINE << "reading alg " << info.alias;
//  //  info.filename = node.getChild("Filename").getValue();
//  //  info.parentAlgorithm = node.getChild("ParentAlgorithm").getValue();
//  //  {
//  //    std::stringstream ss(node.getChild("IDs").getValue());
//  //    unsigned int id;
//  //    while (ss >> id)
//  //      info.ids.push_back(id);
//  //  }
//  //  {
//  //    std::stringstream ss(node.getChild("OutIds").getValue());
//  //    unsigned int id;
//  //    while (ss >> id)
//  //      info.outIds.push_back(id);
//  //  }
//  //  if ( ! info.outIds.empty())
//  //  {
//  //    std::stringstream ss(node.getChild("OutTypes").getValue());
//  //    EnDataType type;
//  //    while (ss >> type)
//  //      info.outTypes.push_back(type);
//  //  }
//  //  if (info.outIds.size() != info.outTypes.size() && info.outTypes.size() == 1)
//  //    info.outTypes.resize(info.outIds.size(), info.outTypes[0]);
//  //  if (info.outIds.size() != info.outTypes.size())
//  //    throw MUK_EXCEPTION("OutIds and OutTypes must have the same size", info.alias.c_str());
//
//  //  if (info.run && saveDate)
//  //  {
//  //    // created date;
//  //    auto t = std::time(nullptr);
//  //    auto tm = *std::localtime(&t);
//  //    std::ostringstream oss;
//  //    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
//  //    info.lastEvalDate = oss.str();
//  //    saveEvaluationDate(S_AppConfigFile, enPreprocessingAlgo, info.lastEvalDate, info.alias);
//  //  }
//  //  else
//  //  {
//  //    info.lastEvalDate = node.getChild("LastEvaluationDate").getValue();
//  //  }
//  //}
//
//  /**
//  */
//  std::vector<XmlNode>::const_iterator  findDiceAlgo(const std::vector<XmlNode>& nodes, const std::string& algoName)
//  {
//    auto iter = std::find_if(nodes.begin(), nodes.end(), [&] (const auto& nd)
//    {
//      return algoName == nd.getChild("Alias").getValue();
//    });
//    if (iter == nodes.end())
//    {
//      std::stringstream ss;
//      ss << "Algorithm for DiceExperiment's algo '" << algoName << "' missing!";
//      throw  MUK_EXCEPTION_SIMPLE(ss.str().c_str());
//    }
//    return iter;
//  }
//}
