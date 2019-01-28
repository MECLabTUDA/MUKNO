#include "private/muk.pch"
#include "private/program_functions.h"
#include "private/program_options.h"
#include "VestibularExperiment.h"
#include "Experiment.h"

#include "MukCommon/MukIO.h"
#include "MukCommon/MukObstacle.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/AppModels.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/SelectionModel.h"

#include "MukEvaluation/CollisionDetectionHandler.h"

#include <Qapplication>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/serialization/vector.hpp>

namespace
{
  using namespace gris::muk;
  namespace fs = boost::filesystem;

  template<typename T> void printElement(std::ofstream& ofs, T t, const int& width)
  {
    const char separator = ' ';
    ofs << std::right << std::setw(width) << std::setfill(separator) << (boost::format("%02.03f") % t).str();
  }

  /** \brief a generic experiment
  */
  class VestibularExperiment : public Experiment
  {
  public:
    VestibularExperiment(const ExperimentInput& input)
      : Experiment(input, enVestibular)
    {
    }

  public:
    //virtual void initialize();
    //void run();
    //bool skipPatient(const PatientData& patientData);

  private:
    virtual void runExperiment(const PatientData& patientData);
    virtual void evalExperiment(const PatientData& patientData);
    virtual void evalExperiments();
  };
}

namespace gris
{
  namespace muk
  {
    /**
    */
    void runVestibularExperiment(const ProgramInput& programInput)
    {
      auto& input = programInput.vestibularData;
      // created result output directories;
      auto pExp = std::make_unique<VestibularExperiment>(input);
      pExp->initialize();
      pExp->run();
      // save log file for debugging
      fs::copy_file( gris::GetGrisLogger().getFilename(), fs::path(input.outputDir) / ScenarioNames[enVestibular] / input.lastEvalDate / "log_run.txt",fs::copy_option::overwrite_if_exists);
    }

    /**
    */
    void evaluateVestibularExperiment(const ProgramInput& programInput)
    {
      const auto& input = programInput.vestibularData;
      {
        auto pExp = std::make_unique<VestibularExperiment>(input);
        pExp->eval();
      }
      fs::copy_file( gris::GetGrisLogger().getFilename(), fs::path(input.outputDir) / ScenarioNames[enVestibular] / input.lastEvalDate / "log_eval.txt",fs::copy_option::overwrite_if_exists);
    }
  }
}

namespace
{
  /**
  */
  void VestibularExperiment::runExperiment(const PatientData& patient)
  {
    LOG_LINE << "===================================================================";
    LOG_LINE << "=== planning SSC and RL access " << patient.id << " ==========";

    AppModels models;
    ApplicationModel& app  = *models.pAppModel;
    auto pScene = app.getScene();
    pScene->setLocalBasePath(mInput.localBasePath);
    PlanningModel&  plan = *models.pPlanningModel;
    const auto patientID = stoi(patient.id.substr(1,2));

    SelectionModel& select = *models.pSelectionModel;

    app.loadScene(patient.sceneFile);
    plan.setPruner("PrunerDummy");

    auto resultDir = fs::path(mInput.outputDir) / ScenarioNames[mExpType] / mInput.lastEvalDate;
    const auto& input  = static_cast<const VestibularExperimentInput&>(mInput);
    const auto& config = input.planningConfig;
    for (const auto& canal : config.accessCanals)
    {
      LOG_LINE << "=== computing GroundTruth paths for access canal: " << canal;
      plan.setActivePathCollection(canal);
      plan.setPlanner(config.planner);
      auto* planner = pScene->getPlanner();
      for (const auto& pair : config.plannerParams)
      {
        planner->setProperty(pair.first, pair.second);
      }
      plan.setInterpolator(config.interpolator);
      auto* inter = pScene->getInterpolator();
      for (const auto& pair : config.interpolatorParams)
      {
        inter->setProperty(pair.first, pair.second);
      }
      {
        plan.clearPaths(canal);
        plan.configurePlanning(canal);
        try
        {
          plan.createPaths(canal, config.planningTime);
        }
        catch (std::exception& e)
        {
          // initial / goal states out of bounds?
          LOG_LINE << "  exception occured path creation (id " << patientID << "):";
          LOG_LINE << e.what();
        }
        const auto& paths = pScene->getPathCollection(canal).getPaths();
        LOG_LINE << "  found paths " << paths.size();
        if ( ! paths.empty())
        {
          const auto& paths = pScene->getPathCollection(canal).getPaths();
          if (select.hasSelection(canal))
          {
            select.makeSelectionValid(canal, paths.size());
          }
          else
          {
            select.makeSelection(canal);
          }
          select.loadPathCollection(canal);
          select.compute();
          auto result = select.getCurrentBest();
          LOG_LINE << "  Largest minimum distance:: " << result.distance.second << " at index " << result.distance.first;

          const auto fn = resultDir / patient.id / (boost::format("%s_path_GT.txt") % canal).str();
          if (!fs::is_directory(fn.parent_path()))
            fs::create_directories(fn.parent_path());
          std::ofstream ofs(fn.generic_string());
          boost::archive::text_oarchive oa(ofs);
          oa << paths[result.distance.first];
        }
      }
      LOG_LINE << "=== computing manual Segmentation paths for access canal: " << canal;
      try
      {
        // first change the data in the collision detector
        for (size_t i(0); i<organs.size(); ++i)
        {
          auto fn = fs::path(config.segmentationDirManual) / patient.id / (organs[i] + ".vtk");
          auto data = loadVtkFile(fn.string());
          app.deleteObstacle(organs[i]);
          auto obs = std::make_shared<MukObstacle>();
          obs->setName(organs[i]);
          obs->setData(data);
          app.addObstacle(obs);
        }
        plan.clearPaths(canal);
        plan.configurePlanning(canal);
        try
        {
          plan.createPaths(canal, config.planningTime);
        }
        catch (std::exception& e)
        {
          // initial / goal states out of bounds?
          LOG_LINE << "  exception occured path creation (id " << patientID << "):";
          LOG_LINE << e.what();
        }
        const auto& paths = pScene->getPathCollection(canal).getPaths();
        LOG_LINE << "  found paths " << paths.size();
        if ( ! paths.empty())
        {
          const auto& paths = pScene->getPathCollection(canal).getPaths();
          if (select.hasSelection(canal))
          {
            select.makeSelectionValid(canal, paths.size());
          }
          else
          {
            select.makeSelection(canal);
          }
          select.loadPathCollection(canal);
          select.compute();
          auto result = select.getCurrentBest();
          LOG_LINE << "  Largest minimum distance:: " << result.distance.second << " at index " << result.distance.first;

          const auto fn = resultDir / patient.id / (boost::format("%s_path_SegManual.txt") % canal).str();
          if (!fs::is_directory(fn.parent_path()))
            fs::create_directories(fn.parent_path());
          std::ofstream ofs(fn.generic_string());
          boost::archive::text_oarchive oa(ofs);
          oa << paths[result.distance.first];
        }
      }
      catch(std::exception& e)
      {
        LOG_LINE << "  failed to compute manual segmentation scene";
        LOG_LINE << e.what();
      }
      LOG_LINE << "=== computing Auto Segmentation paths for access canal: " << canal;
      try
      {
        LOG_LINE << "  Loading from " << (fs::path(config.segmentationDirAuto) / patient.id).string();
        // first change the data in the collision detector
        for (size_t i(0); i<organs.size(); ++i)
        {
          auto fn = fs::path(config.segmentationDirAuto) / patient.id / (organs[i] + ".vtk");
          auto data = loadVtkFile(fn.string());
          app.deleteObstacle(organs[i]);
          auto obs = std::make_shared<MukObstacle>();
          obs->setName(organs[i]);
          obs->setData(data);
          app.addObstacle(obs);
        }
        plan.clearPaths(canal);
        plan.configurePlanning(canal);
        try
        {
          plan.createPaths(canal, config.planningTime);
        }
        catch (std::exception& e)
        {
          // initial / goal states out of bounds?
          LOG_LINE << "  exception occured path creation (id " << patientID << "):";
          LOG_LINE << e.what();
        }
        const auto& paths = pScene->getPathCollection(canal).getPaths();
        LOG_LINE << "  found paths " << paths.size();
        if ( ! paths.empty())
        {
          const auto& paths = pScene->getPathCollection(canal).getPaths();
          if (select.hasSelection(canal))
          {
            select.makeSelectionValid(canal, paths.size());
          }
          else
          {
            select.makeSelection(canal);
          }
          select.loadPathCollection(canal);
          select.compute();
          auto result = select.getCurrentBest();
          LOG_LINE << "  Largest minimum distance:: " << result.distance.second << " at index " << result.distance.first;

          const auto fn = resultDir / patient.id / (boost::format("%s_path_SegAuto.txt") % canal).str();
          if (!fs::is_directory(fn.parent_path()))
            fs::create_directories(fn.parent_path());
          std::ofstream ofs(fn.generic_string());
          boost::archive::text_oarchive oa(ofs);
          oa << paths[result.distance.first];
        }
      }
      catch(std::exception& e)
      {
        LOG_LINE << "  failed to compute auto segmentation scene";
        LOG_LINE << e.what();
      }
    }
  }

  /**
  */
  void VestibularExperiment::evalExperiment(const PatientData& patient)
  {
    const auto rootDir = fs::path(mInput.outputDir) / ScenarioNames[enVestibular] / mInput.lastEvalDate / patient.id;
    LOG_LINE << "   evaluating in directory: " << rootDir.string();

    AppModels models;
    ApplicationModel& app  = *models.pAppModel;
    auto pScene = app.getScene();
    pScene->setLocalBasePath(mInput.localBasePath);
    app.loadScene(patient.sceneFile);

    PlanningModel&  plan = *models.pPlanningModel;
    //const auto patientID = std::stoi(patient.id.substr(1,2));
    SelectionModel& select = *models.pSelectionModel;
    const auto& config = static_cast<const VestibularExperimentInput&>(mInput).planningConfig;
    for (const auto& canal : config.accessCanals)
    {
      plan.configurePlanning(canal);
      plan.setActivePathCollection(canal);
      bool hasGT     = false;
      bool hasManual = false;
      bool hasAuto   = false;
      // first load the paths
      const auto fnGT     = rootDir / (boost::format("%s_path_GT.txt") % canal).str();
      const auto fnManual = rootDir / (boost::format("%s_path_SegManual.txt") % canal).str();
      const auto fnAuto   = rootDir / (boost::format("%s_path_SegAuto.txt") % canal).str();
      if (fs::is_regular_file(fnGT))
      {
        plan.loadPath(canal, fnGT.string());
        hasGT = true;
      }
      if (fs::is_regular_file(fnManual))
      {
        plan.loadPath(canal, fnManual.string());
        hasManual = true;
      }
      if (fs::is_regular_file(fnAuto))
      {
        plan.loadPath(canal, fnAuto.string());
        hasAuto = true;
      }
      // log it
      const auto has1 = hasGT ? "1" : "0";
      const auto has2 = hasManual ? "1" : "0";
      const auto has3 = hasAuto ? "1" : "0";
      LOG_LINE << "  canal " << canal << " of " << patient.id << " has paths (GT, Manual, Auto):" << "(" << has1 << "," << has2 << "," << has3 << ")";
      // save the scene
      auto sceneFile = rootDir / ( patient.id + "_" + canal + "_scene.mukscene");
      pScene->save(sceneFile.string());
      // evaluate the paths on the ground truth (on the obstacles already in the scene)
      int idx(0);
      int idxGT     = hasGT     ? idx++ : -1;
      int idxManual = hasManual ? idx++ : -1;
      int idxAuto   = hasAuto   ? idx++ : -1;
      select.loadPathCollection(canal);
      select.compute();
      auto evalFile = rootDir / ( patient.id + "_" + canal + "_eval.txt");
      auto ofs = std::ofstream(evalFile .string());
      ofs << "has GT path\n"     << hasGT << std::endl;
      if (hasGT)
      {
        const auto& coll = pScene->getPathCollection(canal);
        evalDistances(*pScene, select, canal, idxGT, ofs);
        ofs << std::endl;

        ofs << "has Manual path\n"     << hasManual << std::endl;
        if (hasManual)
        {
          evalDistances(*pScene, select, canal, idxManual, ofs);
          ofs << std::endl;
        }
        ofs << "has auto path\n"     << hasAuto << std::endl;
        if (hasAuto)
        {
          evalDistances(*pScene, select, canal, idxAuto, ofs);
          ofs << std::endl;
        }
      }
    }
  }

  /**
  */
  void VestibularExperiment::evalExperiments()
  {
    const auto rootDir = fs::path(mInput.outputDir) / ScenarioNames[enVestibular] / mInput.lastEvalDate;

    struct CombinedData
    {
      int numDataSets = 0;
      int numGT = 0;
      int numManual = 0;
      int numAuto = 0;

      std::vector<double> minDistGt;
      std::vector<double> minDistMa;
      std::vector<double> minDistAu;

      using ObstacleDistancePair = std::pair<std::string, double>;
      using Pairs = std::vector<ObstacleDistancePair>;
      std::vector<Pairs> distsGT;
      std::vector<Pairs> distsMa;
      std::vector<Pairs> distsAu;
    };

    const auto& config = static_cast<const CochleaExperimentInput&>(mInput).planningConfig;
    for (const auto& canal : config.accessCanals)
    {
      LOG_LINE << "===================" << canal << "====================";
      CombinedData data;
      auto outFile = rootDir / (std::string("combined_results_") + canal + ".txt");
      auto ofs = std::ofstream( outFile.string() );
      for (auto iter = fs::directory_iterator(rootDir); iter != fs::directory_iterator(); ++iter)
      {
        // do not create directories on your own in that folder, or the following code won't work!
        if (!fs::is_directory(*iter))
          continue;
        LOG_LINE << iter->path().string();
        ++data.numDataSets;
        const auto id = iter->path().stem().string();
        const auto fn = rootDir / id / (id + "_" + canal + "_eval.txt");
        auto ifs = ifstream(fn.string());
        std::string line;
        int         i;
        {
          std::getline(ifs, line);
          std::getline(ifs, line);
          i = stoi(line);
          if (!i) // no ground truth path
            continue;
          ++data.numGT;
          // dists
          std::getline(ifs, line);
          std::getline(ifs, line);
          data.minDistGt.push_back(std::stof(line));
          //LOG_LINE << data.minDistGt.back();
          // length
          std::getline(ifs, line);
          std::getline(ifs, line);
          // obstacel dists
          CombinedData::Pairs pairs;
          for (int i(0); i<organs.size(); ++i) // plus the brain
          {
            std::getline(ifs, line);
            const auto obs = line;
            std::getline(ifs, line);
            const auto val  = std::stof(line);
            const auto pair = std::make_pair(obs, val);
            pairs.push_back(pair);
          }
          data.distsGT.push_back(pairs);
          std::getline(ifs, line);
          // manual
          std::getline(ifs, line);
          std::getline(ifs, line);
          i = stoi(line);
          if (i) // has ground truth path
          {
            ++data.numManual;
            // dists
            std::getline(ifs, line);
            std::getline(ifs, line);
            data.minDistMa.push_back(std::stof(line));
            // length
            std::getline(ifs, line);
            std::getline(ifs, line);
            // obstacle dists
            pairs.clear();
            for (int i(0); i<organs.size(); ++i) // plus the brain
            {
              std::getline(ifs, line);
              const auto obs = line;
              std::getline(ifs, line);
              const auto val  = std::stof(line);
              const auto pair = std::make_pair(obs, val);
              pairs.push_back(pair);
            }
            data.distsMa.push_back(pairs);
            std::getline(ifs, line);
          }
          // automatic
          std::getline(ifs, line);
          std::getline(ifs, line);
          i = stoi(line);
          if (i) // no ground truth path
          {
            ++data.numAuto;
            // dists
            std::getline(ifs, line);
            std::getline(ifs, line);
            data.minDistAu.push_back(std::stof(line));
            // length
            std::getline(ifs, line);
            std::getline(ifs, line);
            // obstacel dists
            pairs.clear();
            for (int i(0); i<organs.size(); ++i) // plus the brain
            {
              std::getline(ifs, line);
              const auto obs = line;
              std::getline(ifs, line);
              const auto val  = std::stof(line);
              const auto pair = std::make_pair(obs, val);
              pairs.push_back(pair);
            }
            data.distsAu.push_back(pairs);
            std::getline(ifs, line);
          }
        }
      }
      // collect data
      const auto manualSuccess = data.numManual / (1.0*data.numGT) * 100;
      const auto autoSuccess   = data.numAuto / (1.0*data.numGT) * 100;
      ofs << "data sets " << data.numDataSets << std::endl;
      ofs << "GT        " << data.numGT << std::endl;
      ofs << "% Manual  " << manualSuccess << std::endl;
      ofs << "% Auto    " << autoSuccess << std::endl;
      ofs << std::endl;
      ofs << "min dists" << std::endl;

      const int  nameWidth = 6;
      const int  numWidth  = 15;
      for (int i(0); i<data.minDistGt.size(); ++i)
      {
        printElement(ofs, data.minDistGt[i], numWidth);
        printElement(ofs, data.minDistMa[i], numWidth);
        printElement(ofs, data.minDistAu[i], numWidth);
        ofs << endl;
      }
      //
      const auto N_GT     = data.minDistGt.size();
      const auto N_Manual = data.minDistMa.size();
      const auto N_Auto   = data.minDistAu.size();
      const double safetyDist = canal == "SSC-Access" ? 0.5 : 1.0; // TODO read that in somewhere
      const auto manualDistFailures = std::count_if(data.minDistMa.begin(), data.minDistMa.end(), [&] (double d) { return d < safetyDist; }) / (1.0*N_Manual) ;
      const auto autoDistFailures   = std::count_if(data.minDistAu.begin(), data.minDistAu.end(), [&] (double d) { return d < safetyDist; }) / (1.0*N_Auto) ;
      const auto gtMean             = std::accumulate(data.minDistGt.begin(), data.minDistGt.end(), 0.0, [&] (double in, double d) { return in + 1.0 + std::max(0.0,d); }) / (1.0*N_GT);
      const auto manualMean         = std::accumulate(data.minDistMa.begin(), data.minDistMa.end(), 0.0, [&] (double in, double d) { return in + 1.0 + std::max(0.0,d); }) / (1.0*N_Manual);
      const auto autoMean           = std::accumulate(data.minDistAu.begin(), data.minDistAu.end(), 0.0, [&] (double in, double d) { return in + 1.0 + std::max(0.0,d); }) / (1.0*N_Auto);
      // print as tex tabular
      ofs << std::endl;
      ofs << " & manual & ours \\\\" << "\n";
      ofs << " \\hline\n";
      ofs << " success (\\%) & "     << data.numDataSets/(double)data.numGT << " & " << manualSuccess        << " & " << autoSuccess     << "\\\\\n";
      ofs << " $< d_{min}$ (\\%) & " << "-"                         << " & " << manualDistFailures  << " & " << autoDistFailures << "\\\\\n";
      ofs << " mean min dist & "     << gtMean                  << " & " << manualMean          << " & " << autoMean << "\\\\\n";
      ofs << " &  & \\\\\n";
      ofs << " \\hline\n";
      ofs << " &  & \\\\\n";
      // calculate mean distance to obstacles
      std::map<std::string, double> meanDistsManual;
      std::map<std::string, double> meanDistsAuto;
      for (size_t i(0); i<data.distsMa.size(); ++i)
      {
        const auto& manualDists = data.distsMa[i];
        const auto& autoDists   = data.distsAu[i];
        for (size_t j(0); j<manualDists.size(); ++j)
        {
          if (std::none_of(organs.begin(), organs.end(), [&](const auto& label) { return label == manualDists[j].first; }))
            continue; // no invisible organs
          // initialize, probably not necessary
          if (j == 0)
          {
            meanDistsManual[manualDists[j].first] = 0;
            meanDistsAuto[manualDists[j].first] = 0;
          }
          meanDistsManual[manualDists[j].first] += manualDists[j].second;
          meanDistsAuto[manualDists[j].first]   += autoDists[j].second;
        }
      }
      for (auto& pair : meanDistsManual)
        pair.second /= N_Manual;
      for (auto& pair : meanDistsAuto)
        pair.second /= N_Auto;
      // print mean distance to obstacles
      for (const auto& organ : organs)
      {
        ofs << toAbbreviation(organ) << " & " << (boost::format("%0.2f") % meanDistsManual[organ]).str() << " &" << (boost::format("%0.2f") % meanDistsAuto[organ]).str() << "\\\\\n";
      }
    }
  }
}