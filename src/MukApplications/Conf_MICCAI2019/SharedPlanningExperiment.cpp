#include "private/muk.pch"
#include "SharedPlanningExperiment.h"
#include "PlanningController.h"

#include "MukCommon/PathCollection.h"
#include "MukCommon/MukObstacle.h"
#include "MukCommon/MukProblemDefinition.h"

#include "MukAppModels/AlgorithmModel.h"
#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/SelectionModel.h"

#include "gstd/XmlDocument.h"
#include "gstd/XmlNode.h"

#include <boost/filesystem.hpp>

#include <iostream>
#include <iterator>
#include <numeric>

namespace
{
  using namespace gris;
  using namespace gris::muk;
  namespace fs = boost::filesystem;

  void print(std::ostream& os, const std::vector<double>& v)
  {
    for (size_t i(1); i<v.size(); ++i)
      os << v[i-1] << " ";
    if (!v.empty())
      os << v.back();
  }

  std::istream& operator>> (std::istream& is, std::vector<double>& v)
  {
    std::string s(std::istreambuf_iterator<char>(is), {});
    std::istringstream iss(s);

    std::copy(std::istream_iterator<double>(iss),
      std::istream_iterator<double>(),
      std::back_inserter(v));

    is.clear(std::ios::goodbit | std::ios::eofbit);  // for boost::lexical_cast
    return is;
  }
  
  const std::string postFixRandom    = "RRT";
  const std::string postFixOptimized = "SCO";

  std::vector<std::string> ValidAccesses =
  {
    "Heart-Access",
    "RL-Access",
    "SSC-Access",
  };

  std::vector<std::string> ValidLabels =
  {
    // segthor
    "Aorta",
    "Heart",
    // temporal bone
    "Brain",
    "ExternalAuditoryCanal",
    "FacialNerve",
    "ChordaTympani",
    "JugularVein",
    "Ossicles",
    "SemicircularCanals",
  };

  std::map<std::string, std::string> LabelAbbreviations =
  {
    { "Aorta", "Aorta"} ,
    { "Heart", "Heart"} ,
    { "Brain", "Brain"} ,
    { "ChordaTympani", "ChordaTympani"} ,
    { "ExternalAuditoryCanal", "ExternalAuditoryCanal"} ,
    { "FacialNerve", "FacialNerve"} ,
    { "JugularVein", "JugularVein"} ,
    { "Ossicles", "Ossicles"} ,
    { "SemicircularCanals", "SemicircularCanals"} ,
  };
  
  void load(const XmlNode& node, gstd::DynamicProperty& prop);
  void loadProblemDefinition(MukProblemDefinition& obj, const XmlNode& node, const fs::path& rootDir);
}

namespace gris
{
namespace muk
{
  /**
  */
  std::string SharedPlanningExperiment::rrtKey(const std::string& prefix) const
  {
    return std::string(prefix).append("_").append(postFixRandom);
  }

  /**
  */
  std::string SharedPlanningExperiment::scoKey(const std::string& prefix) const
  {
    return std::string(prefix).append("_").append(postFixOptimized);
  }

  /** \brief computes paths with RRT and dummy optimizer, then copies and optimizes each found path

    Outputs a scene with saved paths for each access canal
    Also writes text file per access canal
    E.g.
      Heart-Access_path_results.txt
    This file includes (for initial and optimized paths) number of paths, best path, minimum distance of best path as well as all minimum distances
    E.g.
      numPaths, idx best one, minimum distance, new line, distances. (first rrt, then sco)
      5 0 2.0424
      2.0424 1.67453 1.33658 1.62814 1.24014
      4 0 3.90816
      3.90816 3.90816 3.90816 3.90816
  */
  void SharedPlanningExperiment::runScene(const std::string& sceneFile, const std::string& id)
  {
    const auto file = fs::path(sceneFile);
    if (file.extension().string() != ".mukscene")
      return;
    LOG_LINE << " -----------------------------------------------------------------------";
    if (skipId(std::stoi(id)))
    {
      LOG_LINE << "    skipping data set " << id;
      return;
    }
    LOG_LINE << "    data set " << id;
    LOG_LINE << " -----------------------------------------------------------------------";
    auto dir = fs::path(mOutputRootDir) / mSubDir / mLastEvalDate / id; // world x
    // create basic models which do the majority of the work
    AppModels models;
    ApplicationModel& app  = *models.pAppModel;
    auto pScene            = app.getScene();
    pScene->setLocalBasePath("../");
    PlanningModel&  plan   = *models.pPlanningModel;
    SelectionModel& select = *models.pSelectionModel;
    app.loadScene( sceneFile );
    mpPlanning->setPlanningModel(plan);
    mpPlanning->setPlanningTime(mPlanningTime);
    mpPlanning->setScene(*pScene);
    for (const auto& accessPath : mAccessPaths)
    {
      const auto& collOrigin = pScene->getPathCollection(accessPath);
      // Create two new path collections, one for each planner
      const auto keyRRT = rrtKey(accessPath);
      const auto keySCO = scoKey(accessPath);
      pScene->insertPathCollection(keyRRT);
      pScene->insertPathCollection(keySCO);
      collOrigin.mirror(pScene->getPathCollection(keyRRT));
      collOrigin.mirror(pScene->getPathCollection(keySCO));
      const auto& refColl = pScene->getPathCollection(accessPath);
      {
        const auto outDir = (dir / "paths").string();
        fs::create_directories(outDir);
        // circular arc planning
        mpPlanning->plan(keyRRT);
        mpPlanning->save(outDir, keyRRT);
        // bezier planning with translation to circular arcs
        mpPlanning->optimize(keyRRT, keySCO);
        mpPlanning->save(outDir, keySCO);
      }
      app.saveScene( (dir / "scene.mukscene").string() );
      // already calculate distances
      {
        const auto fn = std::string(accessPath).append("_path_results.txt");
        std::ofstream ofsResults( (dir / fn).string().c_str() );
        ofsResults << "numPaths, idx best one, minimum distance, new line, distances. (first rrt, then sco)" << std::endl;
        {
          ofsResults << pScene->getPathCollection(keyRRT).getPaths().size() << " ";
          if ( ! pScene->getPathCollection(keyRRT).getPaths().empty())
          {
            plan.setActivePathCollection(keyRRT);
            if (select.hasSelection(keyRRT))
              select.makeSelectionValid(keyRRT, pScene->getPathCollection(keyRRT).getPaths().size());
            else
              select.makeSelection(keyRRT);
            select.loadPathCollection(keyRRT);
            select.compute();
            const auto bestB   = select.getCurrentBest();
            ofsResults << bestB.distance.first << " " << bestB.distance.second << "\n";
          }
          else
          {
            ofsResults << -1 << std::endl;
          }
          const auto& distsRRT = select.getDistances();
          ::print(ofsResults, distsRRT);
          ofsResults << std::endl;
          ofsResults << pScene->getPathCollection(keySCO).getPaths().size() << " ";
          if ( ! pScene->getPathCollection(keySCO).getPaths().empty())
          {
            plan.setActivePathCollection(keySCO);
            if (select.hasSelection(keySCO))
              select.makeSelectionValid(keySCO, pScene->getPathCollection(keySCO).getPaths().size());
            else
              select.makeSelection(keySCO);
            select.loadPathCollection(keySCO);
            select.compute();
            const auto bestB   = select.getCurrentBest();
            ofsResults << bestB.distance.first << " " << bestB.distance.second << "\n";
          }
          else
          {
            ofsResults << -1 << std::endl;
          }
          const auto& distsSCO = select.getDistances();
          ::print(ofsResults, distsSCO);
          ofsResults << "\n";
        }
      }
    }
  }

  /** \brief evaluates a single scene regarding number of paths optimized successful, distance to risk structures, etc...

    Output is written in text files, two per access canal. 
    E.g.
      Heart-Access_RRT_obstacle_results.txt
      Heart-Access_SCO_obstacle_results.txt
     
    Each text file contains minimum distances to risk structures for each path
    E.g.
      Aorta Esophagus Heart Trachea
      1.2292 7.19244 3.23009 9.34631
      1.83258 11.22 5.00461 17.9467
      2.60859 7.641 6.49016 14.5023
      2.0231 8.39921 7.36207 16.8458
      2.51802 7.52167 10.9187 13.8421
      4.27928 7.74613 10.6515 18.6512
  */
  void SharedPlanningExperiment::evaluateScene(const std::string& sceneDir, Evaluations& evals)
  {
    AppModels models;
    ApplicationModel& app  = *models.pAppModel;
    auto& scene            = *app.getScene();
    scene.setLocalBasePath(mLocalBasePath);
    PlanningModel&  plan   = *models.pPlanningModel; // needed for selection
    SelectionModel& select = *models.pSelectionModel;
    const auto dir = fs::path(sceneDir);
    const auto pathdir = dir / "paths"; // contains the paths with filenames according to planner
    if ( ! fs::is_directory(pathdir))
      return;
    const auto fn  = fs::path(sceneDir) / "scene.mukscene";
    app.loadScene(fn.string());
    const auto accesses = scene.getPathKeys();
    LOG_LINE << " === eval: id " << dir.stem().string();
    // accesses is a list like this: Cochlea-Access, SSC-Access, RL-Access, Cochlea-Access_RRT, Cochlea-Access_SCO, SSC-Access_RRT, ....
    std::vector<std::string> dummies;
    for (const auto& access : accesses)
    {
      const auto rrtAccess = rrtKey(access);
      const auto scoAccess = scoKey(access);
      dummies.push_back(rrtAccess);
      dummies.push_back(scoAccess);
      // we just need the original name, skip all that have a postfix
      if (std::any_of(dummies.begin(), dummies.end(), [&] (const auto& str) { return str == access; }))
        continue;
      if (std::none_of(ValidAccesses.begin(), ValidAccesses.end(), [&](const auto& str) { return str == access; }))
        continue;
      auto& eval = evals[access];
      auto& distsRRT      = eval.minDistsRRT;
      auto& distsSCO      = eval.minDistsSCO;
      LOG_LINE << "   access " << access;
      // --------  number of paths --------------------------------------------------------
      const auto nPathsRRT = scene.getPathCollection(rrtAccess).getPaths().size();
      const auto nPathsSCO = scene.getPathCollection(scoAccess).getPaths().size();
      eval.nPathsTotalRRT.push_back(nPathsRRT);
      eval.nPathsTotalSCO.push_back(nPathsSCO);
      LOG_LINE << "     # paths: " << nPathsRRT << " vs. " << nPathsSCO;
      const auto fn = std::string(access).append("_success_rate.txt");
      {
        auto ofs = std::ofstream ( (dir / fn).string().c_str() );
        ofs << "rrt\n" << nPathsRRT << "\n";
        ofs << "sco\n" << nPathsSCO << "\n";
      }
      // -------- distance to risk structures
      // prepare selection model --------------------------------------------------------
      std::vector<std::string> activeObs;
      {
        auto keys = scene.getObstacleKeys();
        for (const auto& key : keys)
        {
          if (scene.getObstacle(key)->getActive())
            activeObs.push_back(key);
        }
        select.setActiveObstacles(activeObs);
      }
      // compute distances for RRT --------------------------------------------------------
      plan.setActivePathCollection(rrtAccess);
      if (select.hasSelection(rrtAccess))
        select.makeSelectionValid(rrtAccess, scene.getPathCollection(rrtAccess).getPaths().size());
      else
        select.makeSelection(rrtAccess);
      select.loadPathCollection(rrtAccess);
      select.setAdvancedOptionsRequested(true); // for calculation of individual obstacle dists
      select.compute();
      {
        const auto fn = std::string(rrtAccess).append("_obstacle_results.txt");
        std::ofstream ofs( (dir / fn).string().c_str() );
        // first the active obs
        if ( ! activeObs.empty())
          ofs << activeObs.front();
        for (size_t i(1); i < activeObs.size(); ++i)
          ofs << " " << activeObs[i];
        ofs << "\n";
        // then values for each path
        const auto vals = select.getMinDistToEachObst();
        for (const auto& path : vals)
        {
          if ( ! path.empty())
            ofs << path.front();
          for (size_t i(1); i < path.size(); ++i)
            ofs << " " << path[i];
          ofs << "\n";
        }
      }
      auto safeMeanDists = [&] (const std::string& access)
      {
        const auto fn = std::string(access).append("_mean_distance_result.txt");
        const auto& paths = scene.getPathCollection(access).getPaths();
        auto ofs = std::ofstream ( (dir / fn).string().c_str() );
        for(size_t i(0); i<paths.size(); ++i)
        {
          const auto coll = scene.getCollisionDetector();
          auto path = paths[i];
          plan.updateInterpolator(i, path);
          const auto dists    = computeDistances(*coll, path.getStates(), 0.0);
          const auto meanDist = std::accumulate(dists.begin(), dists.end(), 0.0) / path.getStates().size();
          ofs << meanDist << "\n";
        }
      };
      // compute mean distance
      safeMeanDists(rrtAccess);
      // compute distances for SCO --------------------------------------------------------
      plan.setActivePathCollection(scoAccess);
      if (select.hasSelection(scoAccess))
        select.makeSelectionValid(scoAccess, scene.getPathCollection(scoAccess).getPaths().size());
      else
        select.makeSelection(scoAccess);
      select.loadPathCollection(scoAccess);
      select.setAdvancedOptionsRequested(true); // for calculation of individual obstacle dists
      select.compute();
      {
        const auto fn = std::string(scoAccess).append("_obstacle_results.txt");
        std::ofstream ofs( (dir / fn).string().c_str() );
        // first the active obs
        if ( ! activeObs.empty())
          ofs << activeObs.front();
        for (size_t i(1); i < activeObs.size(); ++i)
          ofs << " " << activeObs[i];
        ofs << "\n";
        // then values for each path
        const auto vals = select.getMinDistToEachObst();
        for (const auto& path : vals)
        {
          if ( ! path.empty())
            ofs << path.front();
          for (size_t i(1); i < path.size(); ++i)
            ofs << " " << path[i];
          ofs << "\n";
        }
      }
      safeMeanDists(scoAccess);
    }
  }

  /** \brief collects the computed distances to risk structures in every scene and every access canal to get final scores.
  */
  void SharedPlanningExperiment::evaluateTotal(const std::string& dir, Evaluations& evalParams)
  {
    const auto root = fs::path(dir);
    std::ofstream ofsTable  ( (root / "tex_table.txt").string().c_str() ); // every global output is written in this tex table
    // prepare variables
    std::vector<std::string> accesses;
    std::transform(evalParams.begin(), evalParams.end(), std::back_inserter(accesses), [&] (const auto& pair) { return pair.first; } );

    // obstacle dists
    using AccessStatsIndividual = std::map<std::string, Statistics1D>;
    using AccessMapTotal        = std::vector<AccessStatsIndividual>;
    
    std::map<std::string, std::stringstream> lines;

    // individual obstacle dists
    for(size_t j(0); j<accesses.size(); ++j)
    {
      const auto& access = accesses[j];
      if (access == "Cochlea-Access")
        continue;
      AccessMapTotal mapTotalRRT;
      AccessMapTotal mapTotalSCO;
      for(auto itScenes = fs::directory_iterator(root); itScenes != fs::directory_iterator(); ++itScenes)
      {
        if (!fs::is_directory(*itScenes))
          continue;

        const auto& dir = itScenes->path();
        const auto rrt = (dir / rrtKey(access).append("_obstacle_results.txt")).string();
        const auto sco = (dir / scoKey(access).append("_obstacle_results.txt")).string();
        std::ifstream ifsRRT(rrt);
        std::ifstream ifsSCO(sco);
        auto readDists = [&](std::ifstream& ifs) -> AccessStatsIndividual
        {
          AccessStatsIndividual result;
          std::string line;
          std::getline(ifs, line);
          auto ss = std::stringstream(line);
          const auto N_obs = static_cast<size_t>(std::count_if(line.begin(), line.end(), [&](const char c) { return c == ' '; }) + 1);
          std::vector<std::string> obs;
          for(size_t i(0); i<N_obs; ++i)
          {
            std::string key;
            ss >> key;
            obs.push_back(key);
          }
          while(std::getline(ifs,line))
          {
            if (line.empty()) // thats the last one
              break;
            auto ss = std::stringstream(line);
            double v;
            for (size_t i(0); i<N_obs; ++i)
            {
              ss >> v;
              result[obs[i]].mData.push_back(v);
            }
          };
          for (auto& pair : result)
            pair.second.compute();
          return result;
        };
        mapTotalRRT.push_back(readDists(ifsRRT));
        mapTotalSCO.push_back(readDists(ifsSCO));
      }
      // now collect all means and all stds
      std::map<std::string, Statistics1D> rrtMean;
      for(const auto& scene : mapTotalRRT)
        for(const auto& pair : scene)
        rrtMean[pair.first].mData.push_back(pair.second.mMean);
      for(auto& pair : rrtMean)
        pair.second.compute();
      std::map<std::string, Statistics1D> scoMean;
      for(const auto& scene : mapTotalSCO)
        for(const auto& pair : scene)
          scoMean[pair.first].mData.push_back(pair.second.mMean);
      for(auto& pair : scoMean)
        pair.second.compute();
      std::map<std::string, Statistics1D> rrtStd;
      for(const auto& scene : mapTotalRRT)
        for(const auto& pair : scene)
          rrtStd[pair.first].mData.push_back(pair.second.mStdev);
      for(auto& pair : rrtStd)
        pair.second.compute();
      std::map<std::string, Statistics1D> scoStd;
      for(const auto& scene : mapTotalSCO)
        for(const auto& pair : scene)
          scoStd[pair.first].mData.push_back(pair.second.mStdev);
      for(auto& pair : scoStd)
        pair.second.compute();
      
      // ---- now write the table
      for(const auto& pair : rrtMean)
      {
        auto& ss = lines[pair.first];
        if (ss.str().empty())
          ss << pair.first << " & ";
        ss << (boost::format("$%2.02f") % pair.second.mMean).str() << " \\pm " << (boost::format("%2.02f$") % pair.second.mStdev).str() << " & ";
      }
      for(const auto& pair : scoMean)
      {
        auto& ss = lines[pair.first];
        ss << (boost::format("$%2.02f") % pair.second.mMean).str() << " \\pm " << (boost::format("%2.02f$") % pair.second.mStdev).str() << " & ";
      }
    }
    // write table
    for (const auto& line : lines)
    {
      auto str = line.second.str();
      str.pop_back();
      str.pop_back();
      str.append("\\\\");
      LOG_LINE << line.first;
      if (std::none_of(ValidLabels.begin(), ValidLabels.end(), [&](const auto& str) { return str == line.first; }))
        continue;
      ofsTable << str << "\n";
    }
    
    // success rate
    ofsTable << "\n" << "\n";
    std::map<std::string, double> successRates;
    for(size_t j(0); j<accesses.size(); ++j)
    {
      const auto& access = accesses[j];
      if (access == "Cochlea-Access")
        continue;
      std::vector<size_t> nPathsRRT, nPathsSCO;
      for(auto itScenes = fs::directory_iterator(root); itScenes != fs::directory_iterator(); ++itScenes)
      {
        if (!fs::is_directory(*itScenes))
          continue;
        const auto& dir = itScenes->path();
        const auto fn = (dir / std::string(access).append("_success_rate.txt")).string();
        auto ifs = std::ifstream(fn);
        std::string line;
        std::getline(ifs, line);
        std::getline(ifs, line);
        auto ssrrt = std::stringstream(line);
        std::getline(ifs, line);
        std::getline(ifs, line);
        auto sssco = std::stringstream(line);
        size_t nRRT, nSCO;
        ssrrt >> nRRT;
        sssco >> nSCO;
        nPathsRRT.push_back(nRRT);
        nPathsSCO.push_back(nSCO);
      }
      std::vector<double> rates;
      for(size_t i(0); i<nPathsRRT.size(); ++i)
      {
        const auto n1 = nPathsRRT[i];
        const auto n2 = nPathsSCO[i];
        if (n1 == 0)
          continue;
        rates.push_back( (100.0*n2)/n1 );
      }
      const auto mean = std::accumulate(rates.begin(), rates.end(), 0.0) / rates.size();
      successRates[access] = mean;
    }
    ofsTable << "success rate\n";
    for(const auto& pair : successRates)
    {
      ofsTable << pair.first << " & " << pair.second << "\\\\\n";
    }

    // integrated distance
    ofsTable << "\n" << "\n";
    std::map<std::string, double> meanRatesR;
    std::map<std::string, double> meanRatesS;
    for(size_t j(0); j<accesses.size(); ++j)
    {
      const auto& access = accesses[j];
      if (access == "Cochlea-Access")
        continue;
      std::vector<double> meanRRT, meanSCO;
      for(auto itScenes = fs::directory_iterator(root); itScenes != fs::directory_iterator(); ++itScenes)
      {
        if (!fs::is_directory(*itScenes))
          continue;
        const auto& dir = itScenes->path();
        const auto fnR = (dir / std::string(rrtKey(access)).append("_mean_distance_result.txt")).string();
        const auto fnS = (dir / std::string(scoKey(access)).append("_mean_distance_result.txt")).string();
        auto ifsR = std::ifstream(fnR);
        auto ifsS = std::ifstream(fnS);
        auto readMean = [&] (std::ifstream& ifs) -> double
          {
            std::string line;
            std::vector<double> vals;
            while(std::getline(ifs,line))
            {
              if (line.empty())
                continue;
              double d;
              auto ss = std::stringstream(line);
              ss >> d;
              vals.push_back(d);
            };
            auto mean = std::accumulate(vals.begin(), vals.end(), 0.0) / vals.size();
            return mean;
          };
        const auto meanR = readMean(ifsR);
        const auto meanS = readMean(ifsS);
        if (meanR == meanR) // ! nan
          meanRRT.push_back(meanR);
        if (meanS == meanS) // ! nan
          meanSCO.push_back(meanS);
      }
      const auto meanR = std::accumulate(meanRRT.begin(), meanRRT.end(), 0.0) / meanRRT.size();
      const auto meanS = std::accumulate(meanSCO.begin(), meanSCO.end(), 0.0) / meanSCO.size();
      meanRatesR[access] = meanR;
      meanRatesS[access] = meanS;
    }
    ofsTable << "mean distances\n";
    for(const auto& pair : meanRatesR)
    {
      ofsTable << pair.first << " & " << pair.second << " & " << meanRatesS[pair.first] << "\\\\\n";
    }

    // minimum distances
    ofsTable << "\n" << "\n";
    std::map<std::string, double> percentageBetter;
    std::map<std::string, double> meanRrt, meanSco;
    std::map<std::string, double> stdRrt, stdSco;
    for(size_t j(0); j<accesses.size(); ++j)
    {
      const auto& access = accesses[j];
      if (access == "Cochlea-Access")
        continue;
      Statistics1D minDistsR, minDistsS;
      std::vector<bool> scoBetter;
      for(auto itScenes = fs::directory_iterator(root); itScenes != fs::directory_iterator(); ++itScenes)
      {
        if (!fs::is_directory(*itScenes))
          continue;
        const auto& dir = itScenes->path();
        const auto fn = (dir / std::string(access).append("_path_results.txt")).string();
        auto ifs = std::ifstream(fn);
        std::string line;
        std::getline(ifs, line); // documentation
        // rrt
        std::getline(ifs, line);
        auto ss = std::stringstream (line);
        int nRRT, idxRRT;
        double minRRT;
        ss >> nRRT >> idxRRT >> minRRT;
        std::getline(ifs, line); // just the distances
        std::getline(ifs, line);
        ss = std::stringstream (line);
        int nSCO, idxSCO;
        double minSCO;
        ss >> nSCO >> idxSCO >> minSCO;
        minDistsR.mData.push_back(minRRT);
        minDistsS.mData.push_back(minSCO);
        scoBetter.push_back(minRRT < minSCO);
      }
      percentageBetter[access] = std::count_if(scoBetter.begin(), scoBetter.end(), [&] (auto b) { return b; }) / (1.0*scoBetter.size());
      minDistsR.compute();
      minDistsS.compute();
      meanRrt[access] = minDistsR.mMean;
      meanSco[access] = minDistsS.mMean;
      stdRrt[access]  = minDistsR.mStdev;
      stdSco[access]  = minDistsS.mStdev;
    }
    ofsTable << "min distances\n";
    for(const auto& pair : meanRrt)
    {
      const auto& key = pair.first;
      ofsTable << key << "\n";
      ofsTable << "$" << meanRrt[key] << " \\pm " << stdRrt[key] << "$ & " << "$" << meanSco[key] << " \\pm " << stdSco[key] << "\\\\\n";
      ofsTable << "better: " << percentageBetter[key] << "\n";
    }
  }

  /** \brief fills an empty scene with important information of the scenefile

      \param[in,out] scene an empty scene that can be used for inspection of path collections and their Problem Definition parameters.
  */
  void SharedPlanningExperiment::peakScene(const std::string& sceneFilename, MukScene& scene)
  {
    namespace fs = boost::filesystem;
    fs::path p = sceneFilename;
    if (!fs::is_regular_file(sceneFilename))
      throw MUK_EXCEPTION("File does not exist!", p.string().c_str());

    auto doc = gris::XmlDocument::read(sceneFilename.c_str());
    gris::XmlNode root = doc->getRoot();
    {
      auto sceneNode = root.getChild("SurgeryPlanning").getChild("MukScene");
      // auto node = sceneNode.getChild("SceneName");
      auto node = sceneNode.getChild("Obstacles");
      {
        auto obstacles = node.getChildren();
        for (auto& obj : obstacles)
        {
          auto pObj = std::make_shared<MukObstacle>();
          pObj->setName(obj.getChild("Name").getValue());
          const auto* fn = obj.getChild("File").getValue();
          if (fs::path(fn).is_absolute())
          {
            pObj->setFileName(fn);
          }
          else
          {
            auto file_name = fs::path(scene.getLocalBasePath()) / fn;
            pObj->setFileName(file_name.generic_string());
          }
          //pObj->load();
          auto tmp = obj.getChild("Active");
          if (std::string(tmp.getValue()) == "true")
          {
            pObj->setActive(true);
          }
          else
          {
            pObj->setActive(false);
          }
          scene.insertObstacle(pObj);
        }
      }
      node = sceneNode.getChild("PathCollections");
      {
        auto ndCollections = node.getChildren();
        for (auto& ndColl : ndCollections)
        {
          const std::string key = ndColl.getChild("Name").getValue();
          scene.insertPathCollection(key);
          auto& coll = scene.getPathCollection(key);
          ::load(ndColl, coll);
          {
            const auto* key = "InactiveObstacles";
            if (ndColl.hasChild(key))
            {
              auto ndObs = ndColl.getChild(key).getChildren();
              for (const auto& node : ndObs)
                coll.addObstacle(node.getValue());
            }
          }
          auto pProbDef = coll.getProblemDefinition();
          auto ndProbDef = ndColl.getChild("ProblemDefinition");
          loadProblemDefinition(*pProbDef, ndProbDef, fs::path(sceneFilename).parent_path());
          auto tmp = ndColl.getChild("MukPaths");
          auto ndMukPaths = tmp.getChildren();
          for (const auto& ndPath : ndMukPaths)
          {
            //coll.insertPath(MukPath());
          }
        }
      }
    }
  }
}
}


namespace
{
  /**
  */
  void load(const XmlNode& node, gstd::DynamicProperty& prop)
  {
    std::vector<std::string> names;
    prop.getPropertyNames(names);
    for (const auto& name : names)
      if (node.hasChild(name.c_str()))
        prop.setProperty(name, node.getChild(name.c_str()).getValue());
  }
  
  /**
  */
  void loadProblemDefinition(MukProblemDefinition& obj, const XmlNode& node, const fs::path& rootDir)
  {
    auto filename =  rootDir / node.getChild("FileName").getValue();
    if (!fs::is_regular_file(filename))
    {
      throw MUK_EXCEPTION("File not found", filename.generic_string().c_str());
    }
    try
    {
      obj.load(filename.generic_string(), 0);
    }
    catch (boost::archive::archive_exception& e)
    {
      LOG_LINE << "internal error: " << e.what();
      throw MUK_EXCEPTION("Failed to load Problem Definition. Incompatible archive version.", filename.generic_string().c_str());
    }
    // inconsistent save/load:  serializing with boost works only with boosts's own factory.
    // easy manipulation works with xml
    // -> first load with boost, then update possible manual changes in xml
    ::load(node, obj);
    auto ndRegions = node.getChild("StartRegions");
    auto ndChildren = ndRegions.getChildren();
    for (size_t i(0); i<ndChildren.size(); ++i)
    {
      auto& nd = ndChildren[i];
      ::load(nd, obj.getStartRegion(i));
    }
    ndRegions = node.getChild("GoalRegions");
    ndChildren = ndRegions.getChildren();
    for (size_t i(0); i<ndChildren.size(); ++i)
    {
      auto& nd = ndChildren[i];
      ::load(nd, obj.getGoalRegion(i));
    }
  }
}