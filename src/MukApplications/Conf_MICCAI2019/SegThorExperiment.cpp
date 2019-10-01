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

#include <numeric>

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
    , mPostProcessDataSet(false)
    , mAortaToHeartDistanceThreshold(4.0)
    , mAortaMeanRadius(20.0)
  {
    mPlanningTime   = 1.0;
    mName           = "SegThor Experiment";
    mOutputRootDir  = "../results/MukApplications/Conf_MICCAI2019";
    mSubDir         = "SegThor";
    mDataDirSegThor = "D:/DataSets/SegThor_2019";

    auto resourceRootDir   = fs::path("../resources/MukApplications/Conf_MICCAI2019");
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
  void SegThorExperiment::initialize(const XmlNode& node)
  {
    Experiment::initialize(node);
    mpPlanning = std::make_unique<PlanningController>();
    {
      PlanningController::readConfigFile(mPlannerConfigFile, *mpPlanning);
    }
    mAccessPaths = mpPlanning->getAccesses();
  }

  /**
  */
  void SegThorExperiment::print()
  {
    Experiment::print();
    LOG_LINE << "   access paths";
    for (const auto& a : mAccessPaths)
      LOG_LINE << "     " << a;
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
      PlanningController::milliseconds.clear();
      LOG_LINE << " =======================================================================";
      LOG_LINE << " =============== Running SegTHOR Experiment ============================";
      LOG_LINE << " =======================================================================";
      const auto rootDir = fs::path("../resources/MukApplications/Conf_MICCAI2019") / mSubDir / "scenes/";
      for (auto iter = fs::directory_iterator(rootDir); iter != fs::directory_iterator(); ++iter)
      {
        if ( ! fs::is_directory(*iter))
          continue;
        // ../Patient_XX
        for (auto fileIter = fs::directory_iterator(*iter); fileIter != fs::directory_iterator(); ++fileIter)
        {
          if ( ! fs::is_regular_file(*fileIter))
            continue;
          auto ext = fileIter->path().extension().string();
          if (ext != ".mukscene")
            continue;
          const auto fn   = fileIter->path();
          const auto dir  = fileIter->path();
          const auto stem = iter->path().stem().string();
          const auto id   = stem.size() > 2 ? stem.substr(8,2) : "badScene"; // this should then throw
          runScene(fn.string(), id);
          // there is only one scene, but break nonetheless to be sure
          break;
        }
      }
      double mean = std::accumulate(PlanningController::milliseconds.begin(), PlanningController::milliseconds.end(), 0ll);
      LOG_LINE << "mean " << mean;
      LOG_LINE << "mean # " << PlanningController::milliseconds.size();
      mean /= PlanningController::milliseconds.size();
      LOG_LINE << " =======================================================================";
      LOG_LINE << " ========                                               ================";
      LOG_LINE << " ========                                               ================";
      LOG_LINE << " ========                                               ================";
      LOG_LINE << " ========                                               ================";
      LOG_LINE << "                    milliseconds :" << mean;
      LOG_LINE << " ========                                               ================";
      LOG_LINE << " ========                                               ================";
      LOG_LINE << " =======================================================================";
      LOG_LINE << " =======================================================================";
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
    Evaluations evals;
    for (auto iterWorld = fs::directory_iterator(root); iterWorld != fs::directory_iterator(); ++iterWorld)
    {
      evaluateScene(iterWorld->path().string(), evals);
    }
    evaluateTotal(root.string(), evals);
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
    ./root/train/ Patient_01/GT.nii
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
    const auto resourceRootDir   = fs::path("../resources/MukApplications/Conf_MICCAI2019");
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