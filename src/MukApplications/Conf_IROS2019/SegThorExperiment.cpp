#include "private/muk.pch"
#include "SegThorExperiment.h"
#include "PlanningController.h"

#include "MukCommon/muk_dynamic_property_tools.h"
#include "MukCommon/mukIO.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/vtk_tools.h"

#include "MukImaging/muk_imaging_tools.h"

#include "MukVisualization/PolyDataHandler.h"

#include "MukAppModels/AlgorithmModel.h"
#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/SelectionModel.h"

#include <gstd/XmlDocument.h>
#include <gstd/XmlNode.h>

#include <itkImageFileWriter.h>

#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkCleanPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkExtractSelection.h>
#include <vtkUnstructuredGrid.h>
#include <vtkClipPolyData.h>
#include <vtkSphere.h>

#include <boost/filesystem.hpp>

namespace
{
  namespace fs = boost::filesystem;
  const std::vector<std::string> Aliases = {
    "Esophagus",
    "Heart",
    "Trachea",
    "Aorta",
  };
}

namespace gris
{
namespace muk
{
  /** \brief Constructor. Declares its property set
  */
  SegThorExperiment::SegThorExperiment()
    : mExtractDataSet(false)
    , mCopyBaseScene(false)
    , mPostProcessDataSet(true)
    , mAortaToHeartDistanceThreshold(4.0)
    , mAortaMeanRadius(20.0)
    , mPlanningTime(5.0)
  {
    mName           = "SegThor Experiment";
    mOutputRootDir  = "../results/MukApplications/Conf_IROS2019";
    mSubDir         = "SegThor";
    mDataDirSegThor = "D:/DataSets/SegThor_2019";

    auto resourceRootDir   = fs::path("../resources/MukApplications/Conf_IROS2019");
    mAlgExtractLabels      = (resourceRootDir / mSubDir / "LabelImageToPolyData.alg").string();
    mPlannerConfigFile     = (resourceRootDir / mSubDir / "PlannerParameters.mukcfg").string();

    mProps.declareProperty<bool>("ExtractSegThorDataSet", MUK_D_SET(bool, mExtractDataSet), MUK_D_GET(mExtractDataSet));
    mProps.declareProperty<bool>("PostProcessSegThorDataSet", MUK_D_SET(bool, mPostProcessDataSet), MUK_D_GET(mPostProcessDataSet));
    mProps.declareProperty<bool>("CopyBaseScene", MUK_D_SET(bool, mCopyBaseScene), MUK_D_GET(mCopyBaseScene));
    mProps.declareProperty<std::string>("Directory_SegThor", MUK_D_C_SET(std::string, mDataDirSegThor), MUK_D_C_GET(std::string, mDataDirSegThor));
    mProps.declareProperty<std::string>("LabelExtractionAlgorithm", MUK_D_C_SET(std::string, mAlgExtractLabels), MUK_D_C_GET(std::string, mAlgExtractLabels));

    mProps.declareProperty<double>("PlanningTime",           MUK_D_SET(double, mPlanningTime),   MUK_D_GET(mPlanningTime));
    mProps.declareProperty<std::string>("PlannerConfigFile", MUK_D_C_SET(std::string, mPlannerConfigFile), MUK_D_C_GET(std::string, mPlannerConfigFile));
  }

  /**
  */
  void SegThorExperiment::run()
  {
    if (mExtractDataSet)
    {
      LOG_LINE << " =======================================================================";
      LOG_LINE << " ===================== extracting data sets ============================";
      LOG_LINE << " =======================================================================";
      extractSegThor();
    }
    if (mPostProcessDataSet)
    {
      LOG_LINE << " =======================================================================";
      LOG_LINE << " ===================== cropping data sets ==============================";
      LOG_LINE << " =======================================================================";
      postProcessSegThor();
    }
    if (mCopyBaseScene)
    {
      LOG_LINE << " =======================================================================";
      LOG_LINE << " ====================== Copy base scene ================================";
      LOG_LINE << " =======================================================================";
      copyBaseScene();
    }
    if (mRun)
    {
      LOG_LINE << " =======================================================================";
      LOG_LINE << " =============== Running SegTHOR Experiment ============================";
      LOG_LINE << " =======================================================================";

      auto pPlanning = std::make_unique<PlanningController>();
      {
        PlanningController::readConfigFile(mPlannerConfigFile, *pPlanning);
      }
      const auto rootScenes = fs::path("../resources/MukApplications/Conf_IROS2019") / mSubDir / "scenes/";
      for (auto iter = fs::directory_iterator(rootScenes); iter != fs::directory_iterator(); ++iter)
      {
        if ( ! fs::is_directory(*iter))
          continue;
        LOG_LINE << " -----------------------------------------------------------------------";
        const auto id = iter->path().stem().string().substr(8,2);
        if (skipId(std::stoi(id)))
        {
          LOG_LINE << "    skipping data set " << id;
          continue;
        }
        LOG_LINE << "    data set " << id;
        // create basic models which do the majority of the work
        AppModels models;
        ApplicationModel& app  = *models.pAppModel;
        auto pScene            = app.getScene();
        pScene->setLocalBasePath("../");
        PlanningModel&  plan   = *models.pPlanningModel;
        SelectionModel& select = *models.pSelectionModel;
        const auto sceneName   = (iter->path() / "scene.mukscene").string();
        app.loadScene( sceneName );
        // everything of this scene will be placed here
        auto dir = fs::path(mOutputRootDir) / mSubDir / mLastEvalDate / iter->path().stem(); // .../patient_id
        // Create two new path collections, one for each planner
        const auto& collOrigin = pScene->getPathCollection(PlanningController::Key_Origin_SegThor);
        pScene->insertPathCollection(PlanningController::Key_A_SegThor);
        pScene->insertPathCollection(PlanningController::Key_B_SegThor);
        collOrigin.mirror(pScene->getPathCollection(PlanningController::Key_A_SegThor));
        collOrigin.mirror(pScene->getPathCollection(PlanningController::Key_B_SegThor));
        // perform initial planning
        pPlanning->setPlanningModel(plan);
        pPlanning->setPlanningTime(mPlanningTime);
        pPlanning->setScene(*pScene);
        const auto& refColl = pScene->getPathCollection(PlanningController::Key_Origin_SegThor);
        {
          const auto outDir = (dir / "paths").string();
          fs::create_directories(outDir);
          // circular arc planning
          pPlanning->setToCircularArcs();
          pPlanning->setActiveKey(PlanningController::Key_A_SegThor);
          pPlanning->plan();
          pPlanning->save(outDir);
          // bezier planning with translation to circular arcs
          pPlanning->setToBezierSplines();
          pPlanning->setActiveKey(PlanningController::Key_B_SegThor);
          pPlanning->plan();
          pPlanning->save(outDir);
        }
        fs::create_directories( dir / "sceneWithPaths");
        app.saveScene( (dir / "sceneWithPaths.mukscene").string() );
        // perform successive replanning
        // these two will be used for iteratively changing the start state
        std::vector<MukState> statesA;
        std::vector<MukState> statesB;
        std::ofstream ofsResults( (dir / "path_results.txt").string().c_str() );
        ofsResults << "numPaths, idx best one, minimum distance (first arcs, then Bezier)" << std::endl;
        {
          ofsResults << pScene->getPathCollection(PlanningController::Key_A_SegThor).getPaths().size() << " ";
          if ( ! pScene->getPathCollection(PlanningController::Key_A_SegThor).getPaths().empty())
          {
            /*plan.setInterpolator(PlanningController::Interpolator_A);
            plan.setActivePathCollection(PlanningController::Key_A_SegThor);
            if (select.hasSelection(PlanningController::Key_A_SegThor))
              select.makeSelectionValid(PlanningController::Key_A_SegThor, pScene->getPathCollection(PlanningController::Key_A_SegThor).getPaths().size());
            else
              select.makeSelection(PlanningController::Key_A_SegThor);
            select.loadPathCollection(PlanningController::Key_A_SegThor);
            select.compute();
            const auto bestA = select.getCurrentBest();
            statesA = pScene->getPathCollection(PlanningController::Key_A_SegThor).getPaths()[bestA.distance.first].getStates();*/
            const auto& paths = pScene->getPathCollection(PlanningController::Key_A_SegThor).getPaths();
            auto iter = std::min_element(paths.begin(), paths.end(), [&](const auto& lhs, const auto& rhs) { return lhs.getGoalDist() < rhs.getGoalDist();  });
            auto idx  = std::distance(paths.begin(), iter);
            auto dist = iter->getGoalDist();
            statesA = pScene->getPathCollection(PlanningController::Key_A_SegThor).getPaths()[idx].getStates();
            //ofsResults << bestA.distance.first << " " << bestA.distance.second << "\n";
            ofsResults << idx << " " << dist << "\n";
          }
          else
            ofsResults << -1 << std::endl;
          ofsResults << std::endl;
          ofsResults << pScene->getPathCollection(PlanningController::Key_B_SegThor).getPaths().size() << " ";
          if ( ! pScene->getPathCollection(PlanningController::Key_B_SegThor).getPaths().empty())
          {
            plan.setInterpolator(PlanningController::Interpolator_B);
            plan.setActivePathCollection(PlanningController::Key_B_SegThor);
            if (select.hasSelection(PlanningController::Key_B_SegThor))
              select.makeSelectionValid(PlanningController::Key_B_SegThor, pScene->getPathCollection(PlanningController::Key_B_SegThor).getPaths().size());
            else
              select.makeSelection(PlanningController::Key_B_SegThor);
            select.loadPathCollection(PlanningController::Key_B_SegThor);
            select.compute();
            const auto bestB   = select.getCurrentBest();
            statesB = pScene->getPathCollection(PlanningController::Key_B_SegThor).getPaths()[bestB.distance.first].getStates();
            ofsResults << bestB.distance.first << " " << bestB.distance.second << "\n";
          }
          else
            ofsResults << -1 << std::endl;
        }
        // replan for Circular Arcs
        {
          // now iteratively go through the states
          const auto N = statesA.size();
          LOG_LINE << "   === Replanning with " << PlanningController::Planner_A << "===";
          LOG_LINE << "   states available: " << N;
          if (N>2)
          {
            for(size_t i(1); i<N-1; ++i)
            {
              // create a new path collection for this step
              auto key = "Replanning_A_" + std::to_string(i);
              LOG_LINE << "   === replanning for : " << key;
              plan.insertPathCollection(key);
              auto& coll    = pScene->getPathCollection(key);
              refColl.mirror(coll);
              auto* startRegion = dynamic_cast<MukStateRegion*>(&coll.getProblemDefinition()->getStartRegion(0));
              if (nullptr == startRegion)
              {
                throw MUK_EXCEPTION("Broken initial problem definition", sceneName.c_str());
              }
              // set the start region to the current state
              const auto startState = statesA[i];
              startRegion->setCenter(startState);
              // prepare planning
              pPlanning->setActiveKey(key);
              plan.setActivePathCollection(key);
              plan.configurePlanning(key);
              const auto outDir = (dir / key).string();
              fs::create_directories(outDir);
              // circular arc planning
              pPlanning->setToCircularArcs();
              pPlanning->plan();
              pPlanning->save(outDir);
              if (coll.getPaths().empty())
              {
                // no replanning possible
                LOG_LINE << "   no planner found a path. abort replanning for " << key;
                break;
              }
            }
          }
        }
        // replan for Bezier Splines
        {
          // now iteratively go through the states
          const auto N = statesB.size();
          LOG_LINE << "   === Replanning with " << PlanningController::Planner_B << "===";
          LOG_LINE << "   states available: " << N;
          if (N>2)
          {
            for(size_t i(1); i<N-1; ++i)
            {
              // create a new path collection for this step
              auto key = "Replanning_B_" + std::to_string(i);
              LOG_LINE << "   === replanning for : " << key;
              plan.insertPathCollection(key);
              auto& coll    = pScene->getPathCollection(key);
              refColl.mirror(coll);
              auto* startRegion = dynamic_cast<MukStateRegion*>(&coll.getProblemDefinition()->getStartRegion(0));
              if (nullptr == startRegion)
              {
                throw MUK_EXCEPTION("Broken initial problem definition", sceneName.c_str());
              }
              // set the start region to the current state
              MukState startState;
              const auto& W1 = statesB[i-1].coords;
              const auto& W2 = statesB[i].coords;
              startState.coords = W1;// 0.5*(W1 + W2); // have to be between the waypoints for proper direction
              startState.tangent = (W2 - W1).normalized();
              startRegion->setCenter(startState);
              // prepare planning
              pPlanning->setActiveKey(key);
              plan.setActivePathCollection(key);
              plan.configurePlanning(key);
              const auto outDir = (dir / key).string();
              fs::create_directories(outDir);
              // bezier planning with translation to circular arcs
              pPlanning->setToBezierSplines();
              pPlanning->plan();
              pPlanning->save(outDir);
              if (coll.getPaths().empty())
              {
                // no replanning possible
                LOG_LINE << "   no planner found a path. abort replanning for " << key;
                break;
              }
            }
          }
        }
        app.saveScene( (dir / "sceneReplanning.mukscene").string() );
      }
    }
  }

  /**
  */
  void SegThorExperiment::evaluate()
  {
    if ( ! mEvaluate)
      return;
    LOG_LINE << " =======================================================================";
    LOG_LINE << " ================= Evaluate SegTHOR data set ===========================";
    LOG_LINE << " =======================================================================";
    if (mLastEvalDate.empty())
      throw MUK_EXCEPTION_SIMPLE("No Evaluation Date available");
    const auto root = fs::path(mOutputRootDir) / mSubDir / mLastEvalDate;
    std::ofstream ofsInitial( (root / "results_initial.txt").string().c_str() );
    std::ofstream ofsTable  ( (root / "tex_table.txt").string().c_str() );
    ofsTable << " \\textbf{Aorta (Real Data)} & Arcs\\cite{fauser2018:planningNonlinearAccess} & Proposed \\\\" << std::endl;
    ofsTable << " \\hline" << std::endl;
    // ---------------------------------------------------------------------------------
    // --------- evaluate initial trajectory
    // ---------------------------------------------------------------------------------
    size_t nNoPathsBothPlanners(0);
    std::vector<double> nArcsTotal; // we compute percentage later, so use doubles
    std::vector<double> nBezierTotal;
    std::vector<double> distsArcsTotal;
    std::vector<double> distsBezierTotal;
    for (auto iterPatient = fs::directory_iterator(root); iterPatient != fs::directory_iterator(); ++iterPatient)
    {
      if ( ! fs::is_directory(*iterPatient))
        continue;
      const auto& dirPatient = iterPatient->path();
      LOG_LINE << " === evaluate initial trajectory of scene " << dirPatient;
      const auto  id       = std::atoi(dirPatient.stem().string().substr(8,2).c_str()); // folders are named "Pateint_xx"
      const auto  dir      = dirPatient / "paths"; // contains the paths with filenames according to planner
      size_t nArcs(0);
      size_t nBezier(0);
      for(auto iter = fs::directory_iterator(dir); iter != fs::directory_iterator(); ++iter)
      {
        if (!fs::is_regular_file(*iter))
          continue;
        const auto& path = iter->path();
        const auto stem = path.stem().string();
        // filename contains either the string Planner_B or Planner_A.
        if (std::string::npos != stem.find(PlanningController::Planner_A))
        {
          ++nArcs;
        }
        else
        {
          ++nBezier;
        }
      }
      nArcsTotal.push_back(nArcs);
      nBezierTotal.push_back(nBezier);

      LOG_LINE << "   paths of " << PlanningController::Planner_A << ": " << nArcs;
      LOG_LINE << "   paths of " << PlanningController::Planner_B << ": " << nBezier;
      // read minimal distance
      std::ifstream ifs( (dirPatient / "path_results.txt").string() );
      {
        const double r = 0.96;
        const double sd = 1.04;
        std::string line;
        std::getline(ifs, line); // just comments
        int nPaths;
        int idxBest;
        double minDist;
        // planner A 
        std::getline(ifs, line);
        std::stringstream ssA(line);
        ssA >> nPaths >> idxBest >> minDist;
        if (nPaths>0 && minDist >= sd)
          distsArcsTotal.push_back(minDist + r);
        // planner B 
        std::getline(ifs, line);
        std::getline(ifs, line);
        std::stringstream ssB(line);
        ssB >> nPaths >> idxBest >> minDist;
        if (nPaths>0 && minDist >= sd)
          distsBezierTotal.push_back(minDist + r);
      }
      // write result
      ofsInitial << id << " " << nArcs << " " << nBezier << "\n\n";
    }
    ofsInitial << "double fail" << nNoPathsBothPlanners << "\n";
    ofsInitial << "total " << "\n";
    const auto N_Total_Arcs   = std::accumulate(nArcsTotal.begin(), nArcsTotal.end(), 0.0);
    const auto N_Total_Bezier = std::accumulate(nBezierTotal.begin(), nBezierTotal.end(), 0.0);
    Statistics1D statsArcs;
    statsArcs.mData   = nArcsTotal;
    statsArcs.compute();
    ofsInitial << N_Total_Arcs << " " << statsArcs.mMean << " " << statsArcs.mStdev << "\n";
    Statistics1D statsBezier;
    statsBezier.mData = nBezierTotal;
    statsBezier.compute();
    ofsInitial << N_Total_Bezier << " " << statsBezier.mMean << " " << statsBezier.mStdev << "\n";
    const auto rateArcs = std::count_if(nArcsTotal.begin(), nArcsTotal.end(), [&] (double d) { return d != 0; }) / (double)nArcsTotal.size() * 100;
    const auto rateBezier = std::count_if(nBezierTotal.begin(), nBezierTotal.end(), [&] (double d) { return d != 0; }) / (double)nBezierTotal.size() * 100;
    const auto rateArcsStr   = (boost::format("%02.01f") % rateArcs).str();
    const auto rateBezierStr = (boost::format("%02.01f") % rateBezier).str();
    ofsTable << " success rate      & " << rateArcsStr << " & " << rateBezierStr << "\\\\" << std::endl;
    const auto numArcStr    = (boost::format("$%02.01f \\pm %02.01f$") % statsArcs.mMean % statsArcs.mStdev).str();
    const auto numBezierStr = (boost::format("$%02.01f \\pm %02.01f$") % statsBezier.mMean % statsBezier.mStdev).str();
    ofsTable << " \\# paths          & " << numArcStr << " & " << numBezierStr << "\\\\" << std::endl;
    auto dt  = mPlanningTime / statsArcs.mMean;
    auto dtp = std::abs(dt - (mPlanningTime / (statsArcs.mMean + statsArcs.mStdev)));
    const auto timeArcStr    = statsArcs.mMean == 0 ? "-" : (boost::format("$%02.01f \\pm %02.01f$") % dt % dtp ).str();
    dt  = (mPlanningTime / statsBezier.mMean);
    dtp = std::abs(dt - (mPlanningTime / (statsBezier.mMean + statsArcs.mStdev)));
    const auto timeBezierStr = statsBezier.mMean == 0 ? "-" : (boost::format("$%02.01f \\pm %02.01f$") % dt % dtp).str();
    ofsTable << " average time (s)  & " << timeArcStr << " & " << timeBezierStr << "\\\\" << std::endl;
    Statistics1D distStatsArcs;
    distStatsArcs.mData   = distsArcsTotal;
    distStatsArcs.compute();
    Statistics1D distStatsBezier;
    distStatsBezier.mData = distsBezierTotal;
    distStatsBezier.compute();
    const auto distsArcStr    = 0 == distStatsArcs.mMean   ? "-" : (boost::format("$%02.01f \\pm %02.01f$") % distStatsArcs.mMean % distStatsArcs.mStdev ).str();
    const auto distsBezierStr = 0 == distStatsBezier.mMean ? "-" : (boost::format("$%02.01f \\pm %02.01f$") % distStatsBezier.mMean % distStatsBezier.mStdev ).str();
    ofsTable << " min distance (mm) & " << distsArcStr << " & " << distsBezierStr << "\\\\" << std::endl;
    //ofsTable << " $\\min ||\\gamma,C_{Obs}||_{\\mathbb{R}^3}$ (mm) & " << numArcStr << " & " << numBezierStr << "&\\\\" << std::endl;
    LOG_LINE << "   total paths of " << PlanningController::Planner_A   << ": " << N_Total_Arcs;
    LOG_LINE << "   total paths of " << PlanningController::Planner_B << ": " << N_Total_Bezier;
    // ---------------------------------------------------------------------------------
    // --------- evaluate replanning
    // ---------------------------------------------------------------------------------
    std::ofstream ofsReplanning( (root / "results_replanning.txt").string().c_str() );
    std::vector<double> successRateA;
    std::vector<double> successRateB;
    for (auto iterPatient = fs::directory_iterator(root); iterPatient != fs::directory_iterator(); ++iterPatient)
    {
      if ( ! fs::is_directory(*iterPatient))
        continue;
      const auto& dirPatient = iterPatient->path();
      LOG_LINE << " === evaluate replanning trajectories of world " << dirPatient;
      const auto  id       = std::atoi(dirPatient.stem().string().substr(8,2).c_str()); // folders are named "Patient_xx"
      // read initial results
      size_t numStatesA;
      size_t numStatesB;
      int    bestIdxA;
      int    bestIdxB;
      {
        std::ifstream ifs( (fs::path(dirPatient) / "path_results.txt").string().c_str() );
        std::string line;
        std::getline(ifs,line); // only comments
        int nPaths;
        double dist;
        {
          std::getline(ifs,line);
          std::stringstream ss(line);
          LOG_LINE << ss.str();
          ss >> nPaths >> bestIdxA >> dist;
        }
        if (bestIdxA >= 0)
        {
          const auto path = loadMukPathFromTxt( (dirPatient / "paths" / (PlanningController::Planner_A + "_" + std::to_string(bestIdxA) + ".txt")).string() );
          numStatesA = path.getStates().size();
        }
        else
        {
          numStatesA = 0;
        }
        {
          std::getline(ifs,line);
          std::getline(ifs,line);
          std::stringstream ss(line);
          LOG_LINE << ss.str();
          ss >> nPaths >> bestIdxB >> dist;
        }
        if (bestIdxB >= 0)
        {
          const auto path = loadMukPathFromTxt( (dirPatient / "paths" / (PlanningController::Planner_B + "_" + std::to_string(bestIdxB) + ".txt")).string() );
          numStatesB = path.getStates().size();
        }
        else
        {
          numStatesB = 0;
        }
      }
      // check size of best path
      LOG_LINE << "   expect " << numStatesA << " paths from " << PlanningController::Planner_A   << ", best index: " << bestIdxA;
      LOG_LINE << "   expect " << numStatesB << " paths from " << PlanningController::Planner_B << ", best index: " << bestIdxB;
      // count replanning
      size_t N_Replanning_A(0);
      size_t N_Replanning_B(0);
      for(auto iterDirRe = fs::directory_iterator(*iterPatient); iterDirRe != fs::directory_iterator(); ++iterDirRe)
      {
        if ( ! fs::is_directory(*iterDirRe))
          continue;
        const auto stem = iterDirRe->path().stem().string();
        if (std::string::npos != stem.find("Replanning_A"))
          ++N_Replanning_A;
        if (std::string::npos != stem.find("Replanning_B"))
          ++N_Replanning_B;
      }
      const auto percentageA = 100* static_cast<double>(N_Replanning_A) / (numStatesA-2); // first and last state do not count to replanning
      const auto percentageB = 100* static_cast<double>(N_Replanning_B) / (numStatesB-2); // first and last state do not count to replanning
      if (numStatesA)
        successRateA.push_back(percentageA);
      if (numStatesB)
        successRateB.push_back(percentageB);
      LOG_LINE << "   found " << N_Replanning_A << "(" << percentageA << "%) replanned trajectories for " << PlanningController::Planner_A;
      LOG_LINE << "   found " << N_Replanning_B << "(" << percentageB << "%) replanned trajectories for " << PlanningController::Planner_B;
    }
    // overall replanning result
    Statistics1D statsReplannnigA;
    statsReplannnigA.mData = successRateA;
    statsReplannnigA.compute();
    Statistics1D statsReplannnigB;
    statsReplannnigB.mData = successRateB;
    statsReplannnigB.compute();
    const auto rateAStr    = statsReplannnigA.mMean != statsReplannnigA.mMean ? "-" : (boost::format("$%02.01f \\pm %02.01f$") % statsReplannnigA.mMean % statsReplannnigA.mStdev ).str();
    const auto rateBStr    = statsReplannnigB.mMean != statsReplannnigB.mMean ? "-" : (boost::format("$%02.01f \\pm %02.01f$") % statsReplannnigB.mMean % statsReplannnigB.mStdev ).str();
    ofsTable << " replanning rate (\\%) & " << rateAStr << " & " << rateBStr << std::endl;
  }

  /** \brief extract vtkPolyData from the SegThor data set.

    unzipping the SegThor data set gives 
      ./root/train/Patient_01/GT.nii
                              Patient_01.nii
                    Patient_02/GT.nii
                              Patient_02.nii

    This function transforms the labels into polydata and saves them in 
      mOutputRootDir / mSubDir / "Data" / Patient_XX / Aorta.vtk
                                                       Esophagus.vtk
                                                       Trachea.vtk
                                                       Heart.vtk
      mOutputRootDir / mSubDir / "Data" / Patient_YY / Aorta.vtk
                                                       Esophagus.vtk
                                                       Trachea.vtk
                                                       Heart.vtk
      etc...                              ....
  */
  void SegThorExperiment::extractSegThor()
  {
    auto algModel = AlgorithmModel();
    // collect files to process
    auto add_files = [&] (const fs::path& dir, const std::string& key, std::vector<std::string>& files)
    {
      for (auto iter = fs::directory_iterator(dir); iter != fs::directory_iterator(); ++iter)
      {
        if ( ! fs::is_directory(iter->path()))
          continue;
        // now we are in ./patient_xx/
        for (auto iterPatient = fs::directory_iterator(iter->path()); iterPatient != fs::directory_iterator(); ++iterPatient)
        {
          if (!fs::is_regular_file(iterPatient->path()))
            continue;
          const auto path = iterPatient->path();
          const auto stem = path.stem().string();
          if (std::string::npos == stem.find(key))
            continue;
          // now only the image file "Gt.nii" should remain
          files.push_back(path.string());
        }
      }
    };

    const auto dir = fs::path(mDataDirSegThor) / "train/";
    const auto aliasLabel = "InputLabelImage";
    const auto propFN     = "Filename";
    // start with the label image and save polydata of the different heart chambers
    std::vector<std::string> labelFiles;
    add_files(dir, "GT", labelFiles);
    LOG_LINE << "loading algorithm " << mAlgExtractLabels;
    algModel.loadAlgorithm(mAlgExtractLabels);
    for (const auto& file : labelFiles)
    {
      // input filenames are of the form 'Patient_xx/GT.nii' -> save stuff under ./Patient_xx/alias.vtk
      const auto patientStr = fs::path(file).parent_path().stem().string();
      const auto patientID  = std::stoi(patientStr.substr(8,2));
      if (skipId(patientID))
      {
        LOG_LINE << "skipping id " << patientID;
        continue;
      }
      LOG_LINE << "processing file " << file;
      // set filename and compute pipeline
      algModel.getAlgorithmByAlias(aliasLabel)->setProperty(propFN, file);
      algModel.update();
      // save poly data
      const auto outdir = fs::path(mOutputRootDir) / mSubDir / "Data" / patientStr;
      fs::create_directories(outdir);
      for (const auto& alias : Aliases)
      {
        const auto fn = outdir / (alias + ".vtk");
        auto writer = make_vtk<vtkPolyDataWriter>();
        writer->SetFileName(fn.string().c_str());
        auto* pData = static_cast<vtkPolyData*>(algModel.getAlgorithmByAlias(alias)->getOutput(0));
        writer->SetInputData(pData);
        writer->Update();
      }
    }
  }

  /** \brief extract vtkPolyData from the SegThor data set.

  unzipping the SegThor data set gives 
  ./root/train/Patient_01/GT.nii
  Patient_01.nii
  Patient_02/GT.nii
  Patient_02.nii

  This function transforms the labels into polydata and saves them in 
  */
  void SegThorExperiment::postProcessSegThor()
  {
    const auto outDir = fs::path(mOutputRootDir) / mSubDir / "Data/";
    for (auto iter = fs::directory_iterator(outDir); iter != fs::directory_iterator(); ++iter)
    {
      if (!fs::is_directory(*iter))
        continue;
      LOG_LINE << "   dir: " << iter->path().string();
      // read
      auto readerA = make_vtk<vtkPolyDataReader>();
      readerA->SetFileName( (iter->path() / "Aorta.vtk").string().c_str() );
      auto readerH = make_vtk<vtkPolyDataReader>();
      readerH->SetFileName( (iter->path() / "Heart.vtk").string().c_str() );
      readerA->Update();
      readerH->Update();
      auto aorta = readerA->GetOutput();
      auto heart = readerH->GetOutput();
      
      // determine max z of heart
      double boundsH[6];
      heart->GetBounds(boundsH);
      double maxZH = boundsH[5];
      // determine center point of first slice of ascending aorta

      /*auto treeA = make_vtk<vtkKdTree>();
      treeA->AddDataSet(heart);
      treeA->Update();*/

      auto N_A = aorta->GetNumberOfPoints();
      std::vector<std::pair<vtkIdType,Vec3d>> points;
      for(vtkIdType i(0); i<N_A; ++i)
      {
        double p[3];
        aorta->GetPoint(i, p);
        if (std::abs(maxZH - p[2]) < mAortaToHeartDistanceThreshold)
          points.push_back(std::make_pair(i,Vec3d(p)));
      }
      // the relevant points of ascending aorta have all smaller y-Values
      std::sort(points.begin(), points.end(), [&](const auto& p, const auto& q) { return p.second.y() < q.second.y(); });
      std::vector<double> dists;
      if (points.size() > 1)
        dists.push_back((points[0].second - points[1].second).squaredNorm());
      for(size_t i(1); i<points.size();++i)
      {
        dists.push_back((points[i-1].second - points[i].second).squaredNorm());
      }
      auto distIter = std::max_element(dists.begin(), dists.end());
      // should point to the first point in the list of the descending aorta, because of the large gap between ascendign and descending part
      Vec3d centerPointAscendingAorta;
      const auto dIter = std::distance(dists.begin(), distIter);
      std::for_each(points.begin(), points.begin() + dIter, [&] (const auto& p) { centerPointAscendingAorta += p.second; });
      centerPointAscendingAorta /= dIter;
      //LOG_LINE << "centerPointAscendingAorta " << centerPointAscendingAorta << " (" << maxZH << ")";
      Vec3d centerPointHeartTop = centerPointAscendingAorta;
      centerPointHeartTop.z() = maxZH;

      double radius(0.0);
      std::for_each(points.begin(), points.begin() + dIter, [&](const auto& p) { radius += (centerPointAscendingAorta - p.second).norm(); });
      radius /= dIter;
      radius += 1.1; // add some margin
      // clipp aorta and save as new vtkPolydata
      {
        auto clipper = make_vtk<vtkClipPolyData>();
        auto sphere  = make_vtk<vtkSphere>();
        sphere->SetCenter(centerPointAscendingAorta.data());
        sphere->SetRadius(radius);
        clipper->SetGenerateClippedOutput(true);
        clipper->SetClipFunction(sphere);
        clipper->SetInsideOut(true);
        clipper->SetInputData(aorta);
        clipper->Update();
        auto cleaner = make_vtk<vtkCleanPolyData>();
        cleaner->SetInputData(clipper->GetClippedOutput());
        cleaner->Update();
        auto writeA = make_vtk<vtkPolyDataWriter>();
        writeA->SetInputData(cleaner->GetOutput());
        writeA->SetFileName( (iter->path() / "Aorta_cropped.vtk").string().c_str());
        writeA->Update();
        /*{
        auto data = make_vtk<vtkPoints>();
        for (int i(0); i < points.size(); ++i)
        data->InsertNextPoint(points[i].second.data());
        auto poly = make_vtk<vtkPolyData>();
        poly->SetPoints(data);
        PolyDataHandler::addVertices(poly);
        auto writeA = make_vtk<vtkPolyDataWriter>();
        writeA->SetInputData(poly);
        writeA->SetFileName( (iter->path() / "Aorta_poly.vtk").string().c_str());
        writeA->Update();
        }*/
      }
      // clipp heart and save as new vtkPolydata
      {
        auto clipper = make_vtk<vtkClipPolyData>();
        auto sphere  = make_vtk<vtkSphere>();
        sphere->SetCenter(centerPointHeartTop.data());
        sphere->SetRadius(radius);
        clipper->SetGenerateClippedOutput(true);
        clipper->SetClipFunction(sphere);
        clipper->SetInsideOut(true);
        clipper->SetInputData(heart);
        clipper->Update();
        auto cleaner = make_vtk<vtkCleanPolyData>();
        cleaner->SetInputData(clipper->GetClippedOutput());
        cleaner->Update();
        auto write = make_vtk<vtkPolyDataWriter>();
        write->SetInputData(cleaner->GetOutput());
        write->SetFileName( (iter->path() / "Heart_cropped.vtk").string().c_str());
        write->Update();
        /*{
        auto data = make_vtk<vtkPoints>();
        for (int i(0); i < points.size(); ++i)
        data->InsertNextPoint(points[i].second.data());
        auto poly = make_vtk<vtkPolyData>();
        poly->SetPoints(data);
        PolyDataHandler::addVertices(poly);
        auto writeA = make_vtk<vtkPolyDataWriter>();
        writeA->SetInputData(poly);
        writeA->SetFileName( (iter->path() / "Aorta_poly.vtk").string().c_str());
        writeA->Update();
        }*/
      }
    }
  }

  /** \brief Copies the base scene (patient 01) into the result folder and creates new ones for the remaining patients
    
      Warning: overwrites existing scenes. Only needed during development of the experiments.
  */
  void SegThorExperiment::copyBaseScene()
  {
    const auto resourceRootDir   = fs::path("../resources/MukApplications/Conf_IROS2019");
    const auto baseSceneResource = (resourceRootDir / mSubDir / "scene.mukscene").string();
    const auto outDir            = fs::path(mOutputRootDir) / mSubDir / "Data/";
    for (auto iter = fs::directory_iterator(outDir); iter != fs::directory_iterator(); ++iter )
    {
      if (!fs::is_directory(iter->path()))
        continue;
      const auto dir  = iter->path().string();
      const auto stem = fs::path(dir).stem().string();
      if (std::string::npos == stem.find("Patient"))
        continue;
      const auto id = stem.substr(8, 2);
      LOG_LINE << "processing scene " << id;
      const auto fn = fs::path(dir) / "scene.mukscene";
      fs::copy_file(baseSceneResource, fn, fs::copy_option::overwrite_if_exists);
      // copy probdef folder
      auto probDefSrc    = fs::path(baseSceneResource).parent_path() / fs::path(baseSceneResource).filename().stem();
      auto probDefTarget = fn.parent_path() / fn.filename().stem();
      LOG_LINE << "   copy " << probDefSrc << "\n     to\n   " << probDefTarget;
      fs::create_directory(probDefTarget);
      for (auto it = fs::directory_iterator(probDefSrc); it!=fs::directory_iterator(); ++it)
      {
        if (!fs::is_regular_file(*it))
          continue;
        fs::copy_file(*it, probDefTarget / it->path().filename(), fs::copy_option::overwrite_if_exists);
      }
      auto pDoc = gris::XmlDocument::read(fn.string());
      const auto children = pDoc->getRoot().getChild("SurgeryPlanning").getChild("MukScene").getChild("Obstacles").getChildren();
      for(auto& node : children)
      {
        // there are only the 4 obstacles of the SegThor label images
        auto fileNode = node.getChild("File");
        auto path     = fs::path(fileNode.getValue());
        auto newPath  = path.parent_path().parent_path() / (boost::format("Patient_%02d") % id ).str() / path.filename();
        LOG_LINE << "   " << newPath.string();
        fileNode.setValue(newPath.string());
      }
      gris::XmlDocument::save(fn.string(), *pDoc);
    }
  }
}
}