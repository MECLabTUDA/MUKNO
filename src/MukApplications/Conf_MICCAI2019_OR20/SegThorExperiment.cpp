#include "private/muk.pch"
#include "SegThorExperiment.h"

#include "private/CombinedStatistics.h"
#include "private/custom_streams.h"

#include "MukCommon/gris_property_streams.h"
#include "MukCommon/gris_property_tools.h"
#include "MukCommon/muk_dynamic_property_tools.h"
#include "MukCommon/vtk_tools.h"
#include "MukCommon/MukIO.h"

#include "MukCommon/PathCollection.h"
#include "MukCommon/MukObstacle.h"
#include "MukCommon/MukProblemDefinition.h"

#include "MukImaging/MukImagingIO.h"
#include "MukImaging/muk_imaging_tools.h"

#include "MukEvaluation/SegmentationEvaluator.h"
#include "MukEvaluation/statistics.h"

#include "MukAppModels/AlgorithmModel.h"
#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/SelectionModel.h"

#include "MukPdmWrapper/PasmCorrespondenceEstablisher.h"
#include "MukPdmWrapper/PasmMeshExtractor.h"
#include "MukPdmWrapper/PasmSsmGenerator.h"
#include "MukPdmWrapper/PdmAppearanceModelTrainer.h"

#include <pdmSample.h>

#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>

#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataWriter.h>
#include <vtkSphere.h>
#include <vtkParametricEllipsoid.h>

#include <vtkCylinder.h>
#include <vtkExtractGeometry.h>
#include <vtkImplicitBoolean.h>
#include <vtkPlane.h>
#include <vtkSphere.h>
#include <vtkSphereSource.h>
#include <vtkShrinkFilter.h>
#include <vtkClipPolyData.h>
#include <vtkCleanPolyData.h>

#include <boost/filesystem.hpp>

namespace
{
  namespace fs = boost::filesystem;

  const std::string& labelNameFromLabelValue(const std::vector<std::pair<std::string, int>>& pairs, int value)
  {
    static std::string InvalidLabel = "INVALID_LABEL";
    auto iter = std::find_if(pairs.begin(), pairs.end(), [&](const auto& pair) { return pair.second == value; });
    if (iter != pairs.end())
      return iter->first;
    else
      return InvalidLabel;
  }
}

namespace gris
{
namespace muk
{
  const std::string SegThorExperiment::SubSetDir1 = "Subset1";
  const std::string SegThorExperiment::SubSetDir2 = "Subset2";

  /** \brief
  */
  SegThorExperiment::SegThorExperiment()
    : mTransformDataSet(false)
    , mBuildSubset1(false)
    , mBuildSubset2(false)
    , mBuildSurfaces(false)
    , mBuildCorrespondences(false)
    , mBuildSSMs(false)
    , mBuildTrainingFiles(false)
    , mBuildAAMs(false)
    , mUnetToPolydata(false)
    , mPredictPasm(false)
    , mCombinePasm(false)

    , mPlanOnGT(false)
    , mPlanOnUnet(false)
    , mPlanOnShape(false)
    , mCreatePolyData(false)
    , mCutSurfaces(false)
    , mCreatePaths(false)

    , mEvalSegmentation(false)
    , mComputeMetrics(false)
    , mComputeUnetMetric(false)
    , mComputePasmMetric(false)
    , mWriteMetricTable(false)

    , mEvalPlanning(false)
    , mComputeUnetPlanning(false)
    , mComputePasmPlanning(false)
    , mWritePlanningTable(false)
  {
    mLabels = {
      {"Esophagus",1},
      {"Heart",2},
      {"Trachea",3},
      {"Aorta",4},
    };

    mActiveLabels = { 1, 2, 3, 4 };
    mKnownLabels  = {1, 2, 3, 4};
    mSubset1 = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    mSubset2 = {21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40};

    mName           = "SegThor";
    mSubDir         = mName;
    mOutputRootDir  = "../results/MukApplications/Conf_MICCAI2019_OR20";
    mResourceDir    = "../resources/MukApplications/Conf_MICCAI2019_OR20";
    mUnetToPasmInitAlgo = (fs::path(mResourceDir) / mName / "UnetToPASMInit.alg").string();
    mPasmAlgo           = (fs::path(mResourceDir) / mName / "PASM.alg").string();
    mLabelToPolyDataAlgo  = (fs::path(mResourceDir) / mName / "LabelImageToPolyData.alg").string();
    mCutSurfaceConfigFile = (fs::path(mResourceDir) / mName / "cutting.txt").string();
    mOriginalDataBasePath = "D:/DataSets/SegThor_2019/";
    mDataBasePath         = "D:/DataSets/SegThor_2019_mhd/";
    mPasmOutputDir        = "Pasm_Predictions";
    mPasmOutputIndividualSubDir = "Individual";
    mPlanningDataDir = "PlanningData";
    mPlanningResultDir = "PlanningResults";
    mPlanningDirGT   = "GT";
    mPlanningDirUnet = "Unet";
    mPlanningDirSR   = "ShapeReg";
      
    mProps.declareProperty<bool>("BuildDataSet",            MUK_D_SET(bool, mTransformDataSet), MUK_D_GET(mTransformDataSet));
    mProps.declareProperty<bool>("BuildSubset1",            MUK_D_SET(bool, mBuildSubset1), MUK_D_GET(mBuildSubset1));
    mProps.declareProperty<bool>("BuildSubset2",            MUK_D_SET(bool, mBuildSubset2), MUK_D_GET(mBuildSubset2));
    // ASM training
    mProps.declareProperty<bool>("BuildSurfaces",           MUK_D_SET(bool, mBuildSurfaces), MUK_D_GET(mBuildSurfaces));
    mProps.declareProperty<bool>("BuildCorrespondences",    MUK_D_SET(bool, mBuildCorrespondences), MUK_D_GET(mBuildCorrespondences));
    mProps.declareProperty<bool>("BuildSSMs",               MUK_D_SET(bool, mBuildSSMs), MUK_D_GET(mBuildSSMs));
    mProps.declareProperty<bool>("BuildTrainingFiles",      MUK_D_SET(bool, mBuildTrainingFiles), MUK_D_GET(mBuildTrainingFiles));
    mProps.declareProperty<bool>("BuildAAMs",               MUK_D_SET(bool, mBuildAAMs), MUK_D_GET(mBuildAAMs));
    // segmentation
    mProps.declareProperty<bool>("UnetToPolydata",          MUK_D_SET(bool, mUnetToPolydata), MUK_D_GET(mUnetToPolydata));
    mProps.declareProperty<bool>("PredictPasm",             MUK_D_SET(bool, mPredictPasm), MUK_D_GET(mPredictPasm));
    // planning
    mProps.declareProperty<bool>("PlanOnGT",                MUK_D_SET(bool, mPlanOnGT), MUK_D_GET(mPlanOnGT));
    mProps.declareProperty<bool>("PlanOnUnet",              MUK_D_SET(bool, mPlanOnUnet), MUK_D_GET(mPlanOnUnet));
    mProps.declareProperty<bool>("PlanOnShapeReg",          MUK_D_SET(bool, mPlanOnShape), MUK_D_GET(mPlanOnShape));
    mProps.declareProperty<bool>("CreatePolyData",          MUK_D_SET(bool, mCreatePolyData), MUK_D_GET(mCreatePolyData));
    mProps.declareProperty<bool>("CutSurfaces",             MUK_D_SET(bool, mCutSurfaces), MUK_D_GET(mCutSurfaces));
    mProps.declareProperty<bool>("CreatePaths",            MUK_D_SET(bool, mCreatePaths), MUK_D_GET(mCreatePaths));
    // flags for segmentation evaluation
    mProps.declareProperty<bool>("EvalSegmentation",        MUK_D_SET(bool, mEvalSegmentation), MUK_D_GET(mEvalSegmentation));
    mProps.declareProperty<bool>("EvalComputeMetrics",      MUK_D_SET(bool, mComputeMetrics),   MUK_D_GET(mComputeMetrics));
    mProps.declareProperty<bool>("EvalComputeMetricUnet",   MUK_D_SET(bool, mComputeUnetMetric),   MUK_D_GET(mComputeUnetMetric));
    mProps.declareProperty<bool>("EvalComputeMetricPasm",   MUK_D_SET(bool, mComputePasmMetric),   MUK_D_GET(mComputePasmMetric));
    mProps.declareProperty<bool>("EvalWriteTables",         MUK_D_SET(bool, mWriteMetricTable), MUK_D_GET(mWriteMetricTable));
    // flags for planning evaluation
    mProps.declareProperty<bool>("EvalPlanning",            MUK_D_SET(bool, mEvalPlanning), MUK_D_GET(mEvalPlanning));
    mProps.declareProperty<bool>("EvalUnetPlanning",        MUK_D_SET(bool, mComputeUnetPlanning),   MUK_D_GET(mComputeUnetPlanning));
    mProps.declareProperty<bool>("EvalPasmPlanning",        MUK_D_SET(bool, mComputePasmPlanning),   MUK_D_GET(mComputePasmPlanning));
    mProps.declareProperty<bool>("EvalWritePlanningTable",  MUK_D_SET(bool, mWritePlanningTable),   MUK_D_GET(mWritePlanningTable));   
    // other stuff
    mProps.declareProperty<std::vector<int>>("KnownLabels", MUK_D_SET(std::vector<int>, mKnownLabels), MUK_D_GET(mKnownLabels));
    mProps.declareProperty<std::vector<int>>("ActiveLabels", MUK_D_SET(std::vector<int>, mActiveLabels), MUK_D_GET(mActiveLabels));
    mProps.declareProperty<std::vector<int>>("Subset1", MUK_D_SET(std::vector<int>, mSubset1), MUK_D_GET(mSubset1));
    mProps.declareProperty<std::vector<int>>("Subset2", MUK_D_SET(std::vector<int>, mSubset2), MUK_D_GET(mSubset2));
    // algorithms
    mProps.declareProperty<std::string>("UnetToPasmInitAlgorithm",  MUK_D_C_SET(std::string, mUnetToPasmInitAlgo), MUK_D_C_GET(std::string, mUnetToPasmInitAlgo));
    mProps.declareProperty<std::string>("PasmAlgorithm",            MUK_D_C_SET(std::string, mPasmAlgo), MUK_D_C_GET(std::string, mPasmAlgo));
    mProps.declareProperty<std::string>("CombinePasmAlgorithm",     MUK_D_C_SET(std::string, mCombinePasmAlgo), MUK_D_C_GET(std::string, mCombinePasmAlgo));
    mProps.declareProperty<std::string>("LabelToPolyDataAlgorithm",     MUK_D_C_SET(std::string, mLabelToPolyDataAlgo), MUK_D_C_GET(std::string, mLabelToPolyDataAlgo));
    // more config
    mProps.declareProperty<std::string>("CuttingConfigFile",     MUK_D_C_SET(std::string, mCutSurfaceConfigFile), MUK_D_C_GET(std::string, mCutSurfaceConfigFile));
    // directories
    mProps.declareProperty<std::string>("UnetResultDir",    MUK_D_C_SET(std::string, mUnetResultDir), MUK_D_C_GET(std::string, mUnetResultDir));
    mProps.declareProperty<std::string>("UnetExtractedDir", MUK_D_C_SET(std::string, mUnetExtractedDir), MUK_D_C_GET(std::string, mUnetExtractedDir));
    mProps.declareProperty<std::string>("PasmOutputDir",    MUK_D_C_SET(std::string, mPasmOutputDir), MUK_D_C_GET(std::string, mPasmOutputDir));
    mProps.declareProperty<std::string>("PlanningDataDir",  MUK_D_C_SET(std::string, mPlanningDataDir), MUK_D_C_GET(std::string, mPlanningDataDir));
    mProps.declareProperty<std::string>("PlanningResultDir",  MUK_D_C_SET(std::string, mPlanningResultDir), MUK_D_C_GET(std::string, mPlanningResultDir));

    //mProps.declareProperty<size_t>("NumberOfWorlds",        MUK_D_SET(size_t, mNumberOfWorlds)1, MUK_D_GET(mNumberOfWorlds));
    //mProps.declareProperty<double>("PlanningTime",          MUK_D_SET(double, mPlanningTime),   MUK_D_GET(mPlanningTime));
    //mProps.declareProperty<std::string>("SyntheticAnatomyBuilderConfigFile", MUK_D_C_SET(std::string, mSyntheticAnatomyBuilderConfigFile), MUK_D_C_GET(std::string, mSyntheticAnatomyBuilderConfigFile));
    //mProps.declareProperty<std::string>("PlannerConfigFile", MUK_D_C_SET(std::string, mPlannerConfigFile), MUK_D_C_GET(std::string, mPlannerConfigFile));
    mProps.declareProperty<std::string>("DataBasePathOriginal",     MUK_D_C_SET(std::string, mOriginalDataBasePath), MUK_D_C_GET(std::string, mOriginalDataBasePath));
    mProps.declareProperty<std::string>("DataBasePath",     MUK_D_C_SET(std::string, mDataBasePath), MUK_D_C_GET(std::string, mDataBasePath));
  }

  /** \brief
  */
  SegThorExperiment::~SegThorExperiment()
  {
  }

  /** \brief
  */
  void SegThorExperiment::initialize(const XmlNode& node)
  {
    Experiment::initialize(node);
    mPlanningConfig.initialize(node);
  }

  /** \brief
  */
  void SegThorExperiment::run()
  {
    if ( ! mRun)
    {
      LOG_LINE << "'Compute' not activated.";
      return;
    }
    // create new timestamp
    {
      // created date;
      auto t  = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::ostringstream oss;
      oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
      mLastEvalDate = oss.str();
    }
    if (mTransformDataSet)
    {
      LOG_LINE << " =======================================================================";
      LOG_LINE << " ===================== transform data sets =============================";
      LOG_LINE << " =======================================================================";
      transformSegThorDataSet();
    }
    if (mBuildSubset1)
    {
      if (mBuildSurfaces)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== creating surfaces (Subset 1) ====================";
        LOG_LINE << " =======================================================================";
        createSurfaces(enSubset1);
      }
      if (mBuildCorrespondences)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== creating correspondences (Subset 1) =============";
        LOG_LINE << " =======================================================================";
        createCorrespondences(enSubset1);
      }
      if (mBuildSSMs)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== creating SSMs (Subset 1) ========================";
        LOG_LINE << " =======================================================================";
        createSSMModels(enSubset1);
      }
      if (mBuildTrainingFiles)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " =================== creating Training Files (Subset 1)  ===============";
        LOG_LINE << " =======================================================================";
        createTrainingFiles(enSubset1);
      }
      if(mBuildAAMs)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== creating Active Appearance Models (Subset 1) ====";
        LOG_LINE << " =======================================================================";
        createAAMModels(enSubset1);
      }
      if (mUnetToPolydata)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== Unet to Polydata (Subset 1) =====================";
        LOG_LINE << " =======================================================================";
        unetToPolydata(enSubset1);
      }
      if (mPredictPasm)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== predict PASM (Subset 1) =========================";
        LOG_LINE << " =======================================================================";
        predictPasm(enSubset1);
      }
      if (mCombinePasm)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== combine PASM (Subset 1) =========================";
        LOG_LINE << " =======================================================================";
        combinePasm(enSubset1);
      }
      if (mCreatePolyData)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== Create PolyData (Subset 1) ======================";
        LOG_LINE << " =======================================================================";
        if (mPlanOnGT)
          createPolyData(enSubset1, enGt);
        if (mPlanOnUnet)
          createPolyData(enSubset1, enUnet);
        if (mPlanOnShape)
          createPolyData(enSubset1, enShapeReg);
      }
      if (mCutSurfaces)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== Cut Surfaces (Subset 1) =========================";
        LOG_LINE << " =======================================================================";
        if (mPlanOnGT)
          cutSurfaces(enSubset1, enGt);
        if (mPlanOnUnet)
          cutSurfaces(enSubset1, enUnet);
        if (mPlanOnShape)
          cutSurfaces(enSubset1, enShapeReg);
      }
      if (mCreatePaths)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== Create Paths (Subset 1) =========================";
        LOG_LINE << " =======================================================================";
        if (mPlanOnGT)
          createPaths(enSubset1, enGt);
        if (mPlanOnUnet)
          createPaths(enSubset1, enUnet);
        if (mPlanOnShape)
          createPaths(enSubset1, enShapeReg);
      }
    }
    if (mBuildSubset2)
    {
      if (mBuildSurfaces)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== creating surfaces (Subset 2) ====================";
        LOG_LINE << " =======================================================================";
        createSurfaces(enSubset2);
      }
      if (mBuildCorrespondences)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== creating correspondences (Subset 2) =============";
        LOG_LINE << " =======================================================================";
        createCorrespondences(enSubset2);
      }
      if (mBuildSSMs)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== creating SSMs (Subset 2) ========================";
        LOG_LINE << " =======================================================================";
        createSSMModels(enSubset2);
      }
      if (mBuildTrainingFiles)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " =================== creating Training Files (Subset 2)  ===============";
        LOG_LINE << " =======================================================================";
        createTrainingFiles(enSubset2);
      }
      if(mBuildAAMs)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== creating Active Appearance Models (Subset 2) ====";
        LOG_LINE << " =======================================================================";
        createAAMModels(enSubset2);
      }
      if (mUnetToPolydata)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== Unet to Polydata (Subset 2) =====================";
        LOG_LINE << " =======================================================================";
        unetToPolydata(enSubset2);
      }
      if (mPredictPasm)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== predict PASM (Subset 2) =========================";
        LOG_LINE << " =======================================================================";
        predictPasm(enSubset2);
      }
      if (mCombinePasm)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== combine PASM (Subset 2) =========================";
        LOG_LINE << " =======================================================================";
        combinePasm(enSubset2);
      }
      if (mCreatePolyData)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== Create PolyData (Subset 2) ======================";
        LOG_LINE << " =======================================================================";
        if (mPlanOnGT)
          createPolyData(enSubset2, enGt);
        if (mPlanOnUnet)
          createPolyData(enSubset2, enUnet);
        if (mPlanOnShape)
          createPolyData(enSubset2, enShapeReg);
      }
      if (mCutSurfaces)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== Cut Surfaces (Subset 2) =========================";
        LOG_LINE << " =======================================================================";
        if (mPlanOnGT)
          cutSurfaces(enSubset2, enGt);
        if (mPlanOnUnet)
          cutSurfaces(enSubset2, enUnet);
        if (mPlanOnShape)
          cutSurfaces(enSubset2, enShapeReg);
      }
      if (mCreatePaths)
      {
        LOG_LINE << " =======================================================================";
        LOG_LINE << " ===================== Create Paths (Subset 2) =========================";
        LOG_LINE << " =======================================================================";
        if (mPlanOnGT)
          createPaths(enSubset2, enGt);
        if (mPlanOnUnet)
          createPaths(enSubset2, enUnet);
        if (mPlanOnShape)
          createPaths(enSubset2, enShapeReg);
      }
    }
  }

  /** \brief transform original .nii files to .mhd and changes the directory and filename structure

    mOriginalDataBasePath/Patient_01/GT.nii
                                      Patient_01.nii
                          /Patient_02/GT.nii
                                      Patient_02.nii
                          ...

    becomes
    mDataBasePath/dataset1/Patient_01_GT.mhd
                            Patient_01_GT.mha
                            Patient_01_CT.mhd
                            Patient_01_CT.mha
                            Patient_02_GT.mhd
                            Patient_02_GT.mha
                            Patient_02_CT.mhd
                            Patient_02_CT.mha
                            ...
    mDataBasePath/dataset2/Patient_21_GT.mhd
                            Patient_21_GT.mha
                            Patient_21_CT.mhd
                            Patient_21_CT.mha
                            Patient_22_GT.mhd
                            Patient_22_GT.mha
                            Patient_22_CT.mhd
                            Patient_22_CT.mha
                            ...
  */
  void SegThorExperiment::transformSegThorDataSet()
  {
    const auto oldRoot = fs::path(mOriginalDataBasePath);
    const auto newRoot = fs::path(mDataBasePath);
    const auto subset1Dir = newRoot / SubSetDir1;
    if (! fs::is_directory(subset1Dir))
      fs::create_directories(subset1Dir);
    const auto subset2Dir = newRoot / SubSetDir2;
    if (! fs::is_directory(subset2Dir))
      fs::create_directories(subset2Dir);

    auto transformer = [&] (const fs::path& newDir, const std::vector<int>& ids)
    {
      for (auto id : ids)
      {
        if (skipId(id))
        {
          LOG_LINE << "skipping ID " << id;
          continue;
        }
        std::string oldExt = ".nii";
        std::string newExt = ".mhd";
        const auto str = (boost::format("Patient_%02d") % id).str();
        const auto ct = oldRoot / str / (str + oldExt);
        const auto gt = oldRoot / str / ("GT" + oldExt);
        const auto ctNew = newDir / (str + "_CT" + newExt);
        const auto gtNew = newDir / (str + "_GT" + newExt);
        auto reader = make_itk<itk::ImageFileReader<ImageInt3D>>();
        auto writer = make_itk<itk::ImageFileWriter<ImageInt3D>> ();
        writer->SetInput(reader->GetOutput());
        reader->SetFileName(ct.string());
        writer->SetFileName(ctNew.string());
        reader->Update();
        auto matrix = reader->GetOutput()->GetDirection();
        matrix.SetIdentity();
        reader->GetOutput()->SetDirection(matrix);
        writer->Update();

        reader->SetFileName(gt.string());
        reader->Update();
        matrix = reader->GetOutput()->GetDirection();
        matrix.SetIdentity();
        reader->GetOutput()->SetDirection(matrix);
        writer->SetFileName(gtNew.string());
        writer->Update();
      }
    };

    transformer(subset1Dir, mSubset1);
    transformer(subset2Dir, mSubset2);
  }

  /** \brief
  */
  void SegThorExperiment::evaluate()
  {
    if ( ! mEvaluate)
    {
      LOG_LINE << "'Evaluate' not activated.";
      return;
    }
    if(mEvalSegmentation)
    {
      LOG_LINE << " =======================================================================";
      LOG_LINE << " ===================== evaluate segmentation metrics ===================";
      LOG_LINE << " =======================================================================";
      evaluateSegmentation();
    }
    if(mEvalPlanning)
    {
      LOG_LINE << " ===================== evaluate planning metrics =======================";
      LOG_LINE << " =======================================================================";
      LOG_LINE << " =======================================================================";
      evaluatePlanning();
    }
  }

  /** \brief
  */
  void SegThorExperiment::finalize(XmlNode& node)
  {
    Experiment::finalize(node);
    mPlanningConfig.finalize(node);
  }

  /** \brief
  */
  void SegThorExperiment::print()
  {
    Experiment::print();
    LOG_LINE << "   " << "Build Surfaces" << ": " << mBuildSurfaces;
    LOG_LINE << "   " << "Build Correspondence" << ": " << mBuildCorrespondences;
    LOG_LINE << "   " << "Build SSM Models" << ": " << mBuildSSMs;
    LOG_LINE << "   " << "Build AAM Models" << ": " << mBuildAAMs;
    LOG_LINE << "   " << "Build Training Files" << ": " << mBuildTrainingFiles;
    LOG_LINE << "   " << "Build Subset 1" << ": " << mBuildSubset1;
    LOG_LINE << "   " << "Build Subset 2" << ": " << mBuildSubset2;
    LOG_LINE << "   " << "Compute Unet to Polydata" << ": " << mUnetToPolydata;
    LOG_LINE << "   " << "Compute Pasm predictions" << ": " << mPredictPasm;
    LOG_LINE << "   " << "Combine Pasm predictions" << ": " << mCombinePasm;

    //LOG_LINE mPasmOutputDir

    LOG_LINE << "   " << "Data Base Path" << ": " << mDataBasePath;
    LOG      << "   " << "Known Labels"  << ": ";
    for (const auto& pair : mLabels)
      LOG << "(" << pair.first << " " << pair.second << "), ";
    LOG_LINE;
    for (const auto& value : mActiveLabels)
      LOG << value << ", ";
    LOG_LINE;
  }

  // -------------------------------

  /** \brief Extracts smooth surface models from segmentation / label images for all risk structures

  First reads in an xml file defining extraction parameteres as well as in and output files.
  Then proceeds to perform the extraction.
  Output files atm have file name "<PXX>_<risk_structure_name>_surface.vtk"
  */
  void SegThorExperiment::createSurfaces(EnSubSet en)
  {
    const auto N = mLabels.size();
    for (size_t i(0); i < N; ++i)
    {
      const auto& name = mLabels[i].first;
      const auto value = mLabels[i].second;
      if (std::none_of(mActiveLabels.begin(), mActiveLabels.end(), [&](const auto l) { return l == value; }))
        continue;
      LOG_LINE << "  Extracting surface of " << name;
      auto dirConfig = en == enSubset1 ?
        fs::path(mResourceDir) / mName / SubSetDir1
        :
        fs::path(mResourceDir) / mName / SubSetDir2;
      const auto fn = dirConfig / (name + ".mukcfg");
      MeshExtractorBashConfig config;
      config.load(fn.string());
      auto pCreator = std::make_unique<PasmMeshExtractor>();
      {
        const auto M = config.parameters.size();
        for (size_t i(0); i < M; ++i)
        {
          if (!pCreator->hasProperty(config.parameters[i].first))
          {
            LOG_LINE << "    PasmMeshExtractor does not have a property '" << config.parameters[i].first << "'";
          }
          else
          {
            pCreator->setProperty(config.parameters[i].first, config.parameters[i].second);
          }
        }
        if ( ! config.outputMeshFiles.empty())
        {
          auto commonDirectory = fs::path(config.outputMeshFiles.back()).parent_path();
          fs::create_directories(commonDirectory);
        }
        const auto N = config.inputSegmFiles.size();
        auto pWriter = make_vtk<vtkPolyDataWriter>();
        for (size_t i(0); i < N; ++i)
        {
          if (fs::is_regular_file(config.outputMeshFiles[i]))
            continue;
          auto pImage = loadImageInt3D(config.inputSegmFiles[i]);
          pCreator->setImage(*pImage);
          pCreator->setLabel(config.labels[i]);
          pCreator->update();
          LOG_LINE << "    Writing mesh to " << config.outputMeshFiles[i];
          pWriter->SetFileName(config.outputMeshFiles[i].c_str());
          pWriter->SetInputData(pCreator->getOutput());
          pWriter->Update();
        }
      }
    }
  }

  /** \brief Computes pairwise correspondences between the above computed meshes

  First reads in an xml file defining extraction parameteres as well as in and output files.
  Then proceeds to perform the calculation of pairwise corespondences.
  Output files atm have file name "<PXX>_<risk_structure_name>_surface.off"
  */
  void SegThorExperiment::createCorrespondences(EnSubSet en)
  {
    const auto N = mLabels.size();
    for (size_t i(0); i < N; ++i)
    {
      const auto& name = mLabels[i].first;
      const auto value = mLabels[i].second;
      if (std::none_of(mActiveLabels.begin(), mActiveLabels.end(), [&](const auto l) { return l == value; }))
        continue;
      LOG_LINE << "  Building correspondences for " << name;
      auto dirConfig = en == enSubset1 ?
        fs::path(mResourceDir) / mName / SubSetDir1
        :
        fs::path(mResourceDir) / mName / SubSetDir2;
      const auto fn = dirConfig / (name + ".mukcfg");
      auto config = PasmCorrespondenceEstablisher::readConfigFile(fn.string());
      auto meshes = PasmCorrespondenceEstablisher::readMeshes(config);
      auto pCorr = std::make_unique<PasmCorrespondenceEstablisher>();
      // set parameteres
      const auto M = config.parameters.size();
      for (size_t i(0); i < M; ++i)
      {
        if (!pCorr->hasProperty(config.parameters[i].first))
        {
          LOG_LINE << "    PasmCorrespondenceEstablisher does not have a property '" << config.parameters[i].first << "'";
        }
        else
        {
          pCorr->setProperty(config.parameters[i].first, config.parameters[i].second);
        }
      }
      // set meshes
      pCorr->setInputMeshes(std::move(meshes));
      pCorr->update();
      LOG_LINE << "    Writing output ...\n";
      PasmCorrespondenceEstablisher::writeMeshes(config, *pCorr);
      LOG_LINE << "    Finished.\n";
    }
  }

  /** \brief Computes statistical shape models from the above computed correspondence files

  First reads in an xml file defining extraction parameteres as well as in and output files.
  Then proceeds to calculate the SSMs.
  Output files atm have file name "<risk_structure_name>_SSM.mdl"
  */
  void SegThorExperiment::createSSMModels(EnSubSet en)
  {
    const auto N = mLabels.size();
    for (size_t i(0); i < N; ++i)
    {
      const auto& name = mLabels[i].first;
      const auto value = mLabels[i].second;
      if (std::none_of(mActiveLabels.begin(), mActiveLabels.end(), [&](const auto l) { return l == value; }))
        continue;
      LOG_LINE << "  Generating Statistical Shape Model for " << name;
      auto dirConfig = en == enSubset1 ?
        fs::path(mResourceDir) / mName / SubSetDir1
        :
        fs::path(mResourceDir) / mName / SubSetDir2;
      const auto fn = dirConfig / (name + ".mukcfg");
      auto config  = PasmSsmGenerator::readConfigFile(fn.string());
      auto samples = PasmSsmGenerator::readSamples(config);
      auto pSSM    = std::make_unique<PasmSsmGenerator>();
      // set parameteres
      const auto M = config.parameters.size();
      for (size_t i(0); i<M; ++i)
      {
        if (!pSSM->hasProperty(config.parameters[i].first))
        {
          LOG_LINE << "    PasmSsmGenerator does not have a property '" << config.parameters[i].first << "'";
        }
        else
        {
          pSSM->setProperty(config.parameters[i].first, config.parameters[i].second);
        }
      }
      // set meshes
      pSSM->setSamples(std::move(samples));
      pSSM->update();
      PasmSsmGenerator::writeModel(config, *pSSM);
    }
  }

  /** \brief Creates training files required by Active Appearance Models and PASMs.
  */
  void SegThorExperiment::createTrainingFiles(EnSubSet en)
  {
    auto root = fs::path(mOutputRootDir) / mName;
    root/= en == enSubset1 ? SubSetDir1 : SubSetDir2;
    auto rootData = fs::path(mDataBasePath);
    rootData /= en == enSubset1 ? SubSetDir1 : SubSetDir2;
    const auto& listFirst = en == enSubset1 ? mSubset1 : mSubset2;
    for (int j(0); j<mLabels.size(); ++j)
    {
      const auto label = mLabels[j].first;
      const auto value = mLabels[j].second;
      const auto fnTrain = root / label / "ModelFiles" / "training_data_set.txt";
      LOG_LINE << "  writing " << fnTrain.string();
      if ( ! fs::is_directory(fnTrain.parent_path()))
        fs::create_directories(fnTrain.parent_path());
      auto ofTrain = std::ofstream (fnTrain.string());
      for (int i(0); i<listFirst.size(); ++i)
      {
        const auto str = (boost::format("Patient_%02d") % listFirst[i]).str();
        const auto ext = ".mhd";
        const auto ct = str + "_CT" + ext;
        const auto gt = str + "_GT" + ext;
        const auto index = (boost::format("%02d") % listFirst[i]).str();
        ofTrain << listFirst[i] << std::endl;
        const auto lineCorr = root / label / "Correspondences" / (label + "_" + index + ".off");
        ofTrain << lineCorr.string() << std::endl;
        auto lineCT = rootData / ct;
        ofTrain << lineCT.string() << std::endl;
        auto lineSeg = rootData / gt;
        ofTrain << lineSeg.string() << std::endl;
        ofTrain << value << std::endl;
        ofTrain << std::endl;
      }
      ofTrain << std::endl;
    }
  }

  /** \brief Trains an active appearance model for the above statistical shape model.

  First reads in an xml file defining extraction parameteres as well as in and output files.
  Then proceeds to calculate the AAMs.
  Output files atm have file name "<risk_structure_name>_AM.mdl"
  */
  void SegThorExperiment::createAAMModels(EnSubSet en)
  {
    const auto N = mLabels.size();
    for (size_t i(0); i < N; ++i)
    {
      const auto& name = mLabels[i].first;
      const auto value = mLabels[i].second;
      if (std::none_of(mActiveLabels.begin(), mActiveLabels.end(), [&](const auto l) { return l == value; }))
        continue;
      LOG_LINE << "  Training Active Appearance Model for " << name;
      auto dirConfig = en == enSubset1 ?
        fs::path(mResourceDir) / mName / SubSetDir1
        :
        fs::path(mResourceDir) / mName / SubSetDir2;
      const auto fn = dirConfig / (name + ".mukcfg");
      auto algo = std::make_unique<PdmActiveShapeModelTrainer>();
      const auto config = PdmActiveShapeModelTrainer::readConfigFile(fn.string());
      const auto outDir = fs::path(config.outputModelFile).parent_path();
      if ( ! fs::is_directory(outDir))
        fs::create_directories(outDir);
      algo->setConfiguration(config);
      algo->computeModel();
    }
  }

  /** \brief
  */
  void SegThorExperiment::unetToPolydata(EnSubSet en)
  {
    const auto& patients = en == enSubset1 ? mSubset2 : mSubset1; // alternate the subset, because with "enSubset1" activated, we want to predict on subset 2
    for (const auto id : patients)
    { 
      if (skipId(id))
      {
        LOG_LINE << "skipping " << __FUNCTION__ << " for " << id;
        continue;
      }
      const auto index = (boost::format("%02d") % id).str();
      AlgorithmModel model;
      // running algo
      model.loadAlgorithm(mLabelToPolyDataAlgo);
      const auto outputDir = fs::path(mOutputRootDir) / mName / mUnetExtractedDir / index;
      if (!fs::is_directory(outputDir))
      {
        fs::create_directories(outputDir);
      }
      { 
        const auto inputFile = fs::path(mOutputRootDir) / mName / mUnetResultDir / (boost::format("patient_%d_prediction.mhd") % index).str();
        LOG_LINE << "     creating polydatas from " << inputFile.string();
        model.getAlgorithm(0).setProperty("Filename", inputFile.string());
        for(auto labelValue : mKnownLabels)
        {
          const auto& labelName = ::labelNameFromLabelValue(mLabels, labelValue);
          const auto fn = outputDir / (labelName + ".vtk");
          auto* alg = model.getAlgorithmByAlias(labelName);
          alg->setProperty("Filename", fn.string());
        }
        model.update();
      }
    }
  }

  /** \brief
  */
  bool SegThorExperiment::skipLabel(const std::string& label) const
  {
    auto iter = std::find_if(mLabels.begin(), mLabels.end(), [&]  (const auto& pair) { return pair.first == label; });
    if (iter == mLabels.end())
      return true;
    auto labelValue = iter->second;
    return std::none_of(mActiveLabels.begin(), mActiveLabels.end(), [&](const auto& val) { return val == labelValue; });
  }

  /** \brief
  */
  void SegThorExperiment::predictPasm(EnSubSet en)
  {
    const auto& patients = en == enSubset1 ? mSubset2 : mSubset1; // predict on subset 2
    auto dataInputDir    = fs::path(mDataBasePath);
    dataInputDir /= en == enSubset1 ? SubSetDir2: SubSetDir1;   // i.e., read image from subset 2
    auto configDir       = fs::path(mResourceDir) / mName;
    configDir    /= en == enSubset1 ? SubSetDir1 : SubSetDir2;    // i.e., use pasm from subset 1
    const auto outputDir = fs::path(mOutputRootDir) / mName / mPasmOutputDir / mPasmOutputIndividualSubDir;
    if ( ! fs::is_directory(outputDir))
      fs::create_directories(outputDir);
    for (const auto id : patients)
    { 
      if (skipId(id))
      {
        LOG_LINE << "     skip id " << id;
        continue;
      }
      AlgorithmModel model;
      // running algo
      model.loadAlgorithm(mPasmAlgo);
      { 
        const auto index = (boost::format("%02d") % id).str();
        const auto inputCT   = dataInputDir / (boost::format("Patient_%s_CT.mhd") % index).str();
        LOG_LINE << "     running Active Shape Model on\n       " << inputCT.string();
        for(auto labelValue : mKnownLabels)
        {
          const auto& label = ::labelNameFromLabelValue(mLabels, labelValue);
          if (skipLabel(label))
          {
            LOG_LINE << "     skipping " << label;
            continue;
          }
          const auto inputMesh = fs::path(mOutputRootDir) / mName / mUnetExtractedDir / index / (boost::format("%s.vtk") % label).str();
          LOG_LINE << "       predict label " << label << ", using " << inputMesh;
          model.getAlgorithm(0).setProperty("Filename", inputCT.string());
          model.getAlgorithm(2).setProperty("Filename", inputMesh.string());
          const auto configFn =  configDir / (label + ".mukcfg");
          model.getAlgorithm(1).setProperty("ConfigFile", configFn.string());
          std::string str;
          model.getAlgorithm(1).getProperty("ConfigFile", str);
          LOG_LINE << "       using config " << str;
          model.update();
          // writing image
          auto& pasmAlg = model.getAlgorithm(1);
          {
            const auto fn = outputDir / (boost::format("Patient_%s_%s.mhd") % index % label).str();
            LOG_LINE << "  writing to " << fn.string();
            if ( ! (pasmAlg.getOutputType(0) == enImageInt3D) )
            {
              auto str = (boost::format("Algorithm %s, id %d, does not have ImageInt3D output") % mPasmAlgo % id).str();
              throw MUK_EXCEPTION_SIMPLE(str.c_str());
            }
            auto* image = static_cast<ImageInt3D*>(pasmAlg.getOutput(0));
            auto writer = make_itk<itk::ImageFileWriter<ImageInt3D>>();
            writer->SetFileName(fn.string().c_str());
            writer->SetInput(image);
            writer->Update();
          }
          // writing mesh
          {
            const auto fn = outputDir / (boost::format("Patient_%s_%s.vtk") % index % label).str();
            LOG_LINE << "  writing to " << fn.string();
            if ( ! (pasmAlg.getOutputType(1) == enVtkPolyData) )
            {
              auto str = (boost::format("Algorithm %s, id %d, does not have vtkPolyData output") % mPasmAlgo % id).str();
              throw MUK_EXCEPTION_SIMPLE(str.c_str());
            }
            auto* mesh = static_cast<vtkPolyData*>(pasmAlg.getOutput(1));
            auto writer = make_vtk<vtkPolyDataWriter>();
            writer->SetFileName(fn.string().c_str());
            writer->SetInputData(mesh);
            writer->Update();
          }
        }
      }
    }
  }

  /** \brief
  */
  void SegThorExperiment::combinePasm(EnSubSet en)
  {
    const auto& patients = en == enSubset1 ? mSubset2 : mSubset1; // combine on subset 2
    const auto rootOutputDir = fs::path(mOutputRootDir) / mName / mPasmOutputDir;
    for (const auto id : patients)
    { 
      if (skipId(id))
      {
        LOG_LINE << "     skip id " << id;
        continue;
      }
      LOG_LINE << "       combining segmentations for " << id;
      const auto index = (boost::format("%02d") % id).str();
      AlgorithmModel model;
      // running algo
      model.loadAlgorithm(mCombinePasmAlgo);
      {
        for (const auto& labelValue : mKnownLabels)
        {
          const auto& label = ::labelNameFromLabelValue(mLabels, labelValue);
          auto* alg = model.getAlgorithmByAlias(label);
          const auto inputCT   = rootOutputDir / (boost::format("Patient_%s_%s.mhd") % index % label).str();
          alg->setProperty("Filename", inputCT.string());
        }
        model.update();
        // writing image
        auto* alg = model.getAlgorithmByAlias("FinalSegmentation");
        {
          const auto fn = fs::path(mOutputRootDir) / mName / mPasmOutputDir / (boost::format("Patient_%s.mhd") % index).str();
          LOG_LINE << "  writing to " << fn.string();
          if ( ! (alg->getOutputType(0) == enImageInt3D) )
          {
            auto str = (boost::format("Algorithm %s, id %d, does not have ImageInt3D output") % mPasmAlgo % id).str();
            throw MUK_EXCEPTION_SIMPLE(str.c_str());
          }
          auto* image = static_cast<ImageInt3D*>(alg->getOutput(0));
          auto writer = make_itk<itk::ImageFileWriter<ImageInt3D>>();
          writer->SetFileName(fn.string().c_str());
          writer->SetInput(image);
          writer->Update();
        }
      }
    }
  }

  /** \brief
  */ 
  void SegThorExperiment::createPolyData(EnSubSet enSet, EnPlanningPart enPlan)
  {
    const auto& patients     = enSet == enSubset1 ? mSubset2 : mSubset1;
    auto  rootInput = fs::path();
    switch(enPlan)
    {
    case enGt:
      rootInput = fs::path(mDataBasePath);
      rootInput /= enSet == enSubset1 ? SubSetDir2 : SubSetDir1;
      break;
    case enUnet:
      rootInput /= fs::path(mOutputRootDir) / mName / mUnetExtractedDir;
      break;
    case enShapeReg:
      rootInput /= fs::path(mOutputRootDir) / mName / mPasmOutputDir / mPasmOutputIndividualSubDir;
      break;
    }
    auto rootOutput = fs::path(mOutputRootDir) / mName / mPlanningDataDir;
    {
      auto subdir = enPlan == enGt ? mPlanningDirGT : enPlan == enUnet ? mPlanningDirUnet : mPlanningDirSR;
      rootOutput /= subdir;
    }
    for (const auto id : patients)
    { 
      if (skipId(id))
      {
        LOG_LINE << "     skip id " << id;
        continue;
      }
      const auto index  = (boost::format("%02d") % id).str();
      const auto subdir = (boost::format("Patient_%s") % index).str();
      const auto resultDir = rootOutput / subdir;
      if (!fs::is_directory(resultDir))
        fs::create_directories(resultDir);


      if (enPlan == enGt)
      {
        AlgorithmModel model;
        model.loadAlgorithm(mLabelToPolyDataAlgo);
        //const auto  index = (boost::format("%02d") % id).str();
        const auto filename = (boost::format("Patient_%s_GT.mhd") % index).str();
        const auto fn = rootInput / filename;
        LOG_LINE << "  processing " << fn.string();
        model.getAlgorithm(0).setProperty("Filename", fn.string());
        for (const auto& labelValue : mActiveLabels)
        {
          const auto& labelName = ::labelNameFromLabelValue(mLabels, labelValue);
          const auto  fnOut = resultDir / (labelName + ".vtk");
          model.getAlgorithmByAlias(labelName)->setProperty("Filename", fnOut.string());
        };
        model.update();
      }
      else
      {
        const auto inputDir = enPlan == enUnet ? rootInput / index : rootInput;
        for (const auto& labelValue : mActiveLabels)
        {
          const auto& labelName = ::labelNameFromLabelValue(mLabels, labelValue);
          const auto  fnIn  = enPlan == enUnet ?
            inputDir / (labelName + ".vtk")
            :
            inputDir / (boost::format("Patient_%s_%s.vtk") % index % labelName).str();
          const auto  fnOut = resultDir / (labelName + ".vtk");
          LOG_LINE << "copy\n  " << fnIn.string() << "\n  " << fnOut.string();
          fs::copy_file(fnIn, fnOut, fs::copy_option::overwrite_if_exists);
        };
      }
    }
  }

  /** \brief Try cutting the individual segmentation results of PASM
  */
  void SegThorExperiment::cutSurfaces(EnSubSet enSet, EnPlanningPart enPlan)
  {
    const auto& patients = enSet == enSubset1 ? mSubset2 : mSubset1;
    auto rootDir = fs::path(mOutputRootDir) / mName / mPlanningDataDir;
    {
      auto subdir = enPlan == enGt ? mPlanningDirGT : enPlan == enUnet ? mPlanningDirUnet : mPlanningDirSR;
      rootDir /= subdir;
    }
    auto cuttingInfo = readCuttingInfo(mCutSurfaceConfigFile);

    for (const auto id : patients)
    { 
      if (skipId(id))
      {
        LOG_LINE << "     skip id " << id;
        continue;
      }
      const auto index     = (boost::format("%02d") % id).str();
      const auto subdir    = (boost::format("Patient_%02d") % id).str();
      const auto realID    = id;

      for (const auto& labelValue : mActiveLabels)
      {
        const auto& label = ::labelNameFromLabelValue(mLabels, labelValue);
        const auto  stem  = label + ".vtk";
        const auto  fnIn  = rootDir / subdir / (stem);
        const auto  fnOut = rootDir / subdir / (stem);
        LOG_LINE << "   cutting: " << fnIn.string();
        auto surface = loadVtkFile(fnIn.string());
        auto clip = [&] (const Vec3d& position, double radius)
        {
          //auto pClipFunction = make_vtk<vtkSphere>();
          //pClipFunction->SetCenter(const_cast<double*>(position.data()));
          //pClipFunction->SetRadius(radius);
          //
          //auto pClipFilter   = make_vtk<vtkClipPolyData>();
          ////pClipFilter->SetGenerateClippedOutput(true);
          //pClipFilter->SetClipFunction(pClipFunction);
          //pClipFilter->SetInsideOut(false);
          //pClipFilter->SetInputData(surface);
          //pClipFilter->Update();
          //auto pCleanFilter  = make_vtk<vtkCleanPolyData>();
          //pCleanFilter->SetInputData(pClipFilter->GetOutput());
          //pCleanFilter->Update();
          auto cylinder = make_vtk<vtkCylinder>();
          cylinder->SetRadius(radius);
          Vec3d p = position;
          cylinder->SetCenter(p.data());
          cylinder->SetAxis(0, 0, 1);

          Vec3d down = position;
          down.z() -= 2 * radius;
          auto plane1 = make_vtk<vtkPlane>();
          plane1->SetOrigin(down.data());
          plane1->SetNormal(0, 0, 1);
          Vec3d up = position;
          up.z() += 2 * radius;
          auto plane2 = make_vtk<vtkPlane>();
          plane2->SetOrigin(up.data());
          plane2->SetNormal(0, 0, -1);
          auto boolean1 = make_vtk<vtkImplicitBoolean>();
          boolean1->AddFunction(plane1);
          boolean1->AddFunction(plane2);
          boolean1->SetOperationTypeToUnion();

          auto boolean2 = make_vtk<vtkImplicitBoolean>();
          boolean2->AddFunction(cylinder);
          boolean2->AddFunction(boolean1);
          boolean2->SetOperationTypeToDifference();
          auto clip       = make_vtk<vtkClipPolyData>();
          clip->SetGenerateClippedOutput(true);
          clip->SetClipFunction(boolean2);
          clip->SetInsideOut(false);
          clip->SetInputData(surface);
          clip->Update();
          auto clean = make_vtk<vtkCleanPolyData>();
          clean->SetInputData(clip->GetOutput());
          clean->Update();
          surface = clean->GetOutput();
        };
        clip(cuttingInfo[realID].position, cuttingInfo[realID].radius);
        auto writer = make_vtk<vtkPolyDataWriter>();
        writer->SetInputData(surface);
        writer->SetFileName(fnOut.string().c_str());
        writer->Update();
      }
    }
  }

  /** \brief reads in the info from a manually created config file "cutting.txt"

  cutting.txt format (line numbers just for annotation):
  1. some info string
  2. 1001 25.6 350.2 -309.3 48 349 -140.5 1.0 1.0
  3. 1002 -13.5 284.7 -174.8 20.2 289.6 -124.1 1.0 1.0
  ...
  */
  std::map<int, SegThorExperiment::CuttingInfo> SegThorExperiment::readCuttingInfo(const std::string& filename)
  {
    if ( ! fs::is_regular_file(filename))
    {
      throw MUK_EXCEPTION("not a regular file", filename.c_str());
    }
    LOG_LINE << "reading cutting info " << filename;
    std::map<int, SegThorExperiment::CuttingInfo> result;
    auto ifs = std::ifstream(filename);
    std::string line;
    std::getline(ifs, line);
    while(std::getline(ifs ,line))
    {
      auto ss = std::stringstream(line);
      int id;
      ss >> id;
      auto& info = result[id];
      double x, y, z;
      ss >> x >> y >> z;
      info.position = Vec3d(x,y,z);
      ss >> info.radius;
    };
    return result;
  }

  /** \brief evaluates dice score for all patients, once for U-Net, once for shape regularized output
  */
  void SegThorExperiment::evaluateSegmentation()
  {
    SegmentationEvaluator eval;
    std::vector<int> ids;
    std::copy(mSubset1.begin(), mSubset1.end(), std::back_inserter(ids));
    std::copy(mSubset2.begin(), mSubset2.end(), std::back_inserter(ids));
    enum EnType
    {
      enUnet = 0,
      enRegularized = 1,
    };

    auto dataInputDir  = fs::path(mDataBasePath);
    auto outputRootDir = fs::path(mOutputRootDir) / mName;
    if (mComputeMetrics)
    {
      std::vector<int> ks;
      if (mComputeUnetMetric)
        ks.push_back(enUnet);
      if (mComputePasmMetric)
        ks.push_back(enRegularized);
      for (auto k : ks)
      {
        // setup output ofstream
        auto path = outputRootDir;
        path     /= k == enUnet ? mUnetResultDir : mPasmOutputDir;
        const auto filename = (path/ "dice.txt");
        auto ofs = std::ofstream(filename.string().c_str());
        // iterate over data set
        for (const auto& id : ids)
        {
          // some basics
          const auto index = (boost::format("%02d") % id).str();
          const auto str = k == enUnet ? "Unet-" : "Auto-";
          LOG_LINE << "evaluating " << str << "segmentation results for patient " << index;
          // setup filenames for ground truth and segmentation images
          fs::path rootDir = dataInputDir;
          rootDir /= std::any_of(mSubset1.begin(), mSubset1.end(), [&](int i) { return i == id; })
            ? SubSetDir1 : SubSetDir2;
          const auto gtImage = rootDir / (boost::format("Patient_%s_GT.mhd") % index).str();
          if ( ! fs::is_regular_file(gtImage))
          {
            LOG_LINE << "Warning: not a regular file: " << gtImage.string();
            continue;
          }
          rootDir = outputRootDir;
          rootDir /= k == enUnet ? mUnetResultDir : mPasmOutputDir;
          auto segImage = rootDir / ("Patient_" + index + "_prediction.mhd");
          ofs << index << std::endl;
          LOG_LINE << "   compute dice on\n     GT:  " << gtImage.string() << "\n     Seg: " << segImage.string();
          // compute Dice for all labels
          const auto N = mLabels.size();
          for (int i(0); i<N; ++i)
          {
            const auto labelValue = mLabels[i].second;
            const auto labelName  = mLabels[i].first;
            LOG_LINE << "     label " << labelName;
            LOG_LINE << "     value " << labelValue;
            auto readerGT = make_itk<itk::ImageFileReader<ImageInt3D>>();
            readerGT->SetFileName(gtImage.string());
            readerGT->Update();
            auto readerSeg = make_itk<itk::ImageFileReader<ImageInt3D>>();
            if (k == enRegularized)
              segImage = rootDir / mPasmOutputIndividualSubDir / ("Patient_" + index + "_" + labelName + ".mhd");
            readerSeg->SetFileName(segImage.string());
            readerSeg->Update();
            // compute metrics
            auto& img1 = *readerGT->GetOutput();
            auto& img2 = *readerSeg->GetOutput();
            eval.setGroundTruth(img1);
            eval.setSegmentation(img2);
            eval.setLabel(labelValue);
            eval.update();
            const auto& metrics = eval.getMetrics();
            ofs << labelName << " " << labelValue << " " << metrics.dice << " " << metrics.sensitivity << " " << metrics.specificity << " " << metrics.hausdorffDist << std::endl;
          }
        }
      }
    }

    if (mWriteMetricTable)
    {
      using TheBoxMap   = std::map<std::string, BoxPlotData>;
      struct Dice
      {
        Statistics1D dice;
        Statistics1D sensitivity;
        Statistics1D specificity;
        Statistics1D hausdorff;
      };
      using TheStatsMap = std::map<std::string, Dice>;
      TheBoxMap   boxDataUnet;
      TheBoxMap   boxDataAuto;
      TheStatsMap diceDataUnet;
      TheStatsMap diceDataAuto;

      std::string label;
      int     labelValue;
      double  dice;
      double  sensitivity;
      double  specificity;
      double  hausdorff;
      /* read in a txt file that looks like
      1001
      LeftVentricle 500 0.913931 0.841504 0.895451 0.999258
      RightVentricle 600 0.890409 0.802466 0.965791 0.995227
      LeftAtrium 420 0.976167 0.953444 0.983412 0.999225
      RightAtrium 550 0.912194 0.838563 0.876139 0.998874
      LeftMyocardium 205 0.899514 0.81738 0.930978 0.996801
      AscendingAorta 820 0.979498 0.95982 0.9868 0.999373
      PulmonaryArtery 850 0.949855 0.904499 0.944841 0.999138
      1002
      ...
      */
      auto l_read = [&] (std::ifstream& ifs, TheBoxMap& map, TheStatsMap& map2) 
      {
        for (const auto& id : ids)
        {
          std::string nextLine;
          std::getline(ifs, nextLine); // values
          int  id = stoi(nextLine);
          for (int j(0); j<mLabels.size(); ++j)
          {
            std::getline(ifs, nextLine); // values
            auto ss = std::stringstream(nextLine);
            ss >> label;
            ss >> labelValue;
            ss >> dice;
            ss >> sensitivity;
            ss >> specificity;
            ss >> hausdorff;
            if (dice > 0)
            {
              map[label].mData.push_back(dice);
              map2[label].dice.mData.push_back(dice);
              map2[label].sensitivity.mData.push_back(sensitivity);
              map2[label].specificity.mData.push_back(specificity);
              map2[label].hausdorff.mData.push_back(hausdorff);
            }
            else
            {
              map[label].mFailures += 1;
            }
          }
        }
      };
      // just read in back in again for easily readable and modifiable coding
      {
        auto fn  = outputRootDir / mUnetResultDir / "dice.txt";
        auto ifs = std::ifstream(fn.string());
        LOG_LINE << "reading unet dice from " << fn.string();
        l_read(ifs, boxDataUnet, diceDataUnet);

        fn  = outputRootDir / mPasmOutputDir / "dice.txt";
        ifs = std::ifstream(fn.string());
        LOG_LINE << "reading pasm dice from " << fn.string();
        l_read(ifs, boxDataAuto, diceDataAuto);
      }
      // compute stats
      for (auto& pair : boxDataUnet)
        pair.second.compute();
      for (auto& pair : boxDataAuto)
        pair.second.compute();
      for (auto& pair : diceDataUnet)
      {
        pair.second.dice.compute();
        pair.second.sensitivity.compute();
        pair.second.specificity.compute();
        pair.second.hausdorff.compute();
      }
      for (auto& pair : diceDataAuto)
      {
        pair.second.dice.compute();
        pair.second.sensitivity.compute();
        pair.second.specificity.compute();
        pair.second.hausdorff.compute();
      }
      // print metrics in tex tabular form 
      {
        // Unet
        auto l_printTable = [&] (TheStatsMap& map, const fs::path& fn) 
        {
          auto ofsFinal          = std::ofstream(fn.string());
          for (const auto& pair : mLabels)
          {
            const auto& name = pair.first;
            const auto& dice = map[name].dice;
            const auto& sens = map[name].sensitivity;
            const auto& spec = map[name].specificity;
            const auto& haus = map[name].hausdorff;
            const auto w1 = 25;          
            const auto w2 = 8;
            const auto p = 2;
            auto cap = [&] (double d) 
            {
              return (boost::format("%1.02f") % d).str(); 
            };
            ofsFinal << std::left << std::setw(w1) << name  
              << " & " << cap(dice.mMean) << "(" << cap(dice.mStdev) << ")"
              << " & " << cap(sens.mMean) << "(" << cap(sens.mStdev) << ")"
              << " & " << cap(spec.mMean) << "(" << cap(spec.mStdev) << ")"
              << " & " << cap(haus.mMean) << "(" << cap(haus.mStdev) << ")" 
              << "\\\\" << std::endl;
          }
        };
        auto fn1 = outputRootDir / mUnetResultDir / "final_result_tex.txt";
        auto fn2 = outputRootDir / mPasmOutputDir / "final_result_tex.txt";
        l_printTable(diceDataUnet, fn1);
        l_printTable(diceDataAuto, fn2);
      }
    }
  }

  /** \brief Search for paths according to the planning configuration
  */
  void SegThorExperiment::createPaths(EnSubSet enSet, EnPlanningPart enPlan)
  {
    auto basePath = fs::path(mOutputRootDir) / mName / mPlanningDataDir;
    const auto timeStamp = mLastEvalDate;
    switch(enPlan)
    {
    case enGt:       
      basePath /= mPlanningDirGT; 
      mPlanningConfig.timeStampGt = timeStamp;
      break;
    case enUnet:     
      basePath /= mPlanningDirUnet; 
      mPlanningConfig.timeStampUnet = timeStamp;;
      break;
    case enShapeReg: 
      basePath /= mPlanningDirSR;
      mPlanningConfig.timeStampSR = timeStamp;
      break;
    };
    const auto& patients = enSet == enSubset1 ? mSubset2 : mSubset1;
    auto resultDir = fs::path(mOutputRootDir) / mName / mPlanningResultDir;
    {
      auto subdir = enPlan == enGt ? mPlanningDirGT : enPlan == enUnet ? mPlanningDirUnet : mPlanningDirSR;
      resultDir /= subdir;
      resultDir /= timeStamp;
    }
    for (const auto id : patients)
    { 
      if (skipId(id))
      {
        LOG_LINE << "     skip id " << id;
        continue;
      }
      AppModels models;
      ApplicationModel& app  = *models.pAppModel;
      PlanningModel&    plan = *models.pPlanningModel;
      SelectionModel&   select = *models.pSelectionModel;

      const auto patientStr = (boost::format("Patient_%02d") % id).str();

      auto pScene = app.getScene();
      pScene->setLocalBasePath(basePath.string());
      const auto sceneFile = fs::path(mResourceDir) / mName / "planning_scenarios" / patientStr / "scene.mukscene";
      app.loadScene(sceneFile.string());
      const auto& c     = mPlanningConfig;
      const auto& canal = c.accessCanal;
      LOG_LINE << "  computing trajectories for access canal: " << canal;
      plan.setPlanner(c.planner);
      plan.setInterpolator(c.interpolator);
      auto* planner = pScene->getPlanner();
      auto* inter   = pScene->getInterpolator();
      for (const auto& pair : c.plannerParams)
        planner->setProperty(pair.first, pair.second);      
      for (const auto& pair : c.interpolatorParams)
        inter->setProperty(pair.first, pair.second);
      plan.setActivePathCollection(canal);
      plan.clearPaths(canal);
      plan.configurePlanning(canal);
      try
      {
        plan.createPaths(canal, c.planningTime);
      }
      catch (std::exception& e)
      {
        // initial / goal states out of bounds?
        LOG_LINE << "  exception occured path creation (id " << id << "):";
        LOG_LINE << e.what();
      }
      auto& paths = pScene->getPathCollection(canal).getPaths();
      LOG_LINE << "set optimizer to " << c.optimizer;
      plan.setPruner(c.optimizer);
      plan.configurePlanning(canal);
      auto* opti= pScene->getPruner();
      for (const auto& pair : c.optimizerParams)
        opti->setProperty(pair.first, pair.second);
      int counter(0);
      for (size_t i(0); i<paths.size(); ++i)
      {
        auto path = opti->calculate(paths[i]);
        /*for (const auto& pair : mParamsBezierOptimizer2)
          optimizer->setProperty(pair.first, pair.second);
        path = optimizer->calculate(path);*/
        if (opti->valid())
        {
          paths[i] = path;
          ++counter;
        }
      }
      LOG_LINE << "  found paths " << paths.size();
      LOG_LINE << "  optimized   " << counter;
      const auto patientDir = (boost::format("Patient_%02d") % id).str();
      const auto outputDir  = resultDir / patientDir;
      if ( ! fs::is_directory(outputDir))
        fs::create_directories(outputDir);
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
        const auto fn = outputDir / (boost::format("%s_path.txt") % canal).str();
        std::ofstream ofs(fn.generic_string());
        boost::archive::text_oarchive oa(ofs);
        oa << paths[result.distance.first];
      }
      // save for use for visualization
      {
        auto keys = pScene->getObstacleKeys();
        for(const auto& key: keys)
        {
          auto obs = pScene->getObstacle(key);
          auto str = obs->getFileName();
          str = fs::absolute(fs::path(str)).string();
          obs->setFileName(str);
        }
        const auto fn = outputDir / "scene_vis.mukscene";
        LOG_LINE << "  save scene to " << fn.string();
        pScene->save(fn.string());
      }
      // save for use during evaluation
      {
        auto keys = pScene->getObstacleKeys();
        for(const auto& key: keys)
        {
          auto obs = pScene->getObstacle(key);
          auto str = obs->getFileName();
          auto fn  = fs::path(str).filename();
          auto dir = fs::path(str).parent_path().stem().string();
          auto path = fs::path(dir) / fn;
          obs->setFileName(path.string());
        }
        const auto fn = outputDir / "scene_eval.mukscene";
        LOG_LINE << "  save scene to " << fn.string();
        pScene->save(fn.string());
      }
    }
  }

  /** \brief
  */
  SegThorExperiment::PlanningConfiguration::PlanningConfiguration()
    : accessCanal("Heart-Access")
    , planner("Spline-Based-RRT-Connect")
    , optimizer("ConvexBezierSplineOptimizer")
    , interpolator("Yang-Spiral-Interpolator")
    , planningTime(2.0)
  {
    props.declareProperty<std::string>("AccessCanal",   MUK_D_C_SET(std::string, accessCanal), MUK_D_C_GET(std::string, accessCanal));
    props.declareProperty<double>("PlanningTime",       MUK_D_C_SET(double, planningTime), MUK_D_C_GET(double, planningTime));
    props.declareProperty<std::string>("Planner",       MUK_D_C_SET(std::string, planner), MUK_D_C_GET(std::string, planner));
    props.declareProperty<std::string>("Optimizer",     MUK_D_C_SET(std::string, optimizer), MUK_D_C_GET(std::string, optimizer));
    props.declareProperty<std::string>("Interpolator",  MUK_D_C_SET(std::string, interpolator), MUK_D_C_GET(std::string, interpolator));
    props.declareProperty<std::string>("LastRunGt",         MUK_D_C_SET(std::string, timeStampGt), MUK_D_C_GET(std::string, timeStampGt));
    props.declareProperty<std::string>("LastRunUnet",       MUK_D_C_SET(std::string, timeStampUnet), MUK_D_C_GET(std::string, timeStampUnet));
    props.declareProperty<std::string>("LastRunShapeReg",   MUK_D_C_SET(std::string, timeStampSR), MUK_D_C_GET(std::string, timeStampSR));

    plannerParams = 
    {
        {"Calc-Time", "1.0"}
      , {"ConeLength", "45"}
      , {"ConeRadius", "5"}
      , {"MaxChildren", "10"}
      , {"Step-Size", "15.0"}
    };

    for (int i(0); i<plannerParams.size(); ++i)
    {
      auto& value = plannerParams[i].second; // force properties to take a reference? 
      propsPlanner.declareProperty<std::string>(plannerParams[i].first,   MUK_D_C_SET(std::string, value), MUK_D_C_GET(std::string, value));
    }
    /*optimizerParams = 
    {
        {"MaxDistancePenalty", "30"}
      , {"DistanceCost", "true"}
      , {"LengthCost", "false"}
      , {"CurvatureConstraint", "false"}
      , {"WeightLength", "0.1"}
      , {"WeightObstacles", "10"}
      , {"InitialTrustRegionSize", "0.5"}
      , {"MaxTrustRegionSize", "0.1"}
    };*/
    optimizerParams = 
    {
      {"MaxDistancePenalty", "30"}
      , {"DistanceCost", "true"}
      , {"LengthCost", "true"}
      , {"CurvatureConstraint", "true"}
      , {"WeightLength", "0.1"}
      , {"WeightObstacles", "10"}
      , {"InitialTrustRegionSize", "0.5"}
      , {"MaxTrustRegionSize", "0.1"}
    };
    for (int i(0); i<optimizerParams.size(); ++i)
    {
      auto& value = optimizerParams[i].second; // force properties to take a reference? 
      propsOptimizer.declareProperty<std::string>(optimizerParams[i].first,   MUK_D_C_SET(std::string, value), MUK_D_C_GET(std::string, value));
    }
  }

  /** \brief
  */
  void SegThorExperiment::PlanningConfiguration::initialize(const XmlNode& node_)
  {
    const auto name = "Plannning";
    if ( ! node_.hasChild(name))
      return;
    auto node = node_.getChild(name);
    std::vector<std::string> names;
    std::vector<std::string> values;
    props.getPropertyNames(names);
    for(const auto& name: names)
      XmlNodeHelper::read(node, name, props);

    if (node_.hasChild("PlannerParameters"))
    {
      auto plannerNode = node_.getChild("PlannerParameters");
      std::vector<std::string> names;
      std::vector<std::string> values;
      propsPlanner.getPropertyNames(names);
      for(const auto& name: names)
        XmlNodeHelper::read(plannerNode, name, propsPlanner);
    }
    if (node_.hasChild("OptimizerParameters"))
    {
      auto optiNode = node_.getChild("OptimizerParameters");
      std::vector<std::string> names;
      std::vector<std::string> values;
      propsOptimizer.getPropertyNames(names);
      for(const auto& name: names)
        XmlNodeHelper::read(optiNode, name, propsOptimizer);
    }
  }

  /** \brief
  */
  void SegThorExperiment::PlanningConfiguration::finalize(XmlNode& node)
  {
    {
      const auto key = "Plannning";
      XmlNode planningNode = node.hasChild(key) ?
        node.getChild(key)
        :
        node.addChild(key);
      std::vector<std::string> names;
      props.getPropertyNames(names);
      for(const auto& name: names)
      {
        XmlNodeHelper::write(planningNode, name, props);
      }
    }
    {
      const auto key = "PlannerParameters";
      XmlNode plannerNode = node.hasChild(key) ?
        node.getChild(key)
        :
        node.addChild(key);
      std::vector<std::string> names;
      propsPlanner.getPropertyNames(names);
      for(const auto& name : names)
      {
        std::string value;
        propsPlanner.getProperty(name, value);
        XmlNodeHelper::write(plannerNode, name, propsPlanner);
      }
    }
    {
      const auto key = "OptimizerParameters";
      XmlNode optiNode = node.hasChild(key) ?
        node.getChild(key)
        :
        node.addChild(key);
      std::vector<std::string> names;
      propsOptimizer.getPropertyNames(names);
      for(const auto& name : names)
      {
        std::string value;
        propsOptimizer.getProperty(name, value);
        XmlNodeHelper::write(optiNode, name, propsOptimizer);
      }
    }
  }

  /** \brief
  */
  void SegThorExperiment::PlanningConfiguration::print() const
  {
  }

  /** \brief
  */
  void SegThorExperiment::evaluatePlanning()
  {
    const auto rootObstacleDir = fs::path(mOutputRootDir) / mName / mPlanningDataDir;
    const auto obstacleDirGT   = rootObstacleDir / mPlanningDirGT  ;// / mPlanningConfig.timeStampGt;
    const auto obstacleDirUnet = rootObstacleDir / mPlanningDirUnet;// / mPlanningConfig.timeStampUnet;
    const auto obstacleDirSR   = rootObstacleDir / mPlanningDirSR  ;// / mPlanningConfig.timeStampSR;

    auto rootResultDir = fs::path(mOutputRootDir) / mName / mPlanningResultDir;
    const auto resultDirGT   = rootResultDir / mPlanningDirGT   / mPlanningConfig.timeStampGt;
    const auto resultDirUnet = rootResultDir / mPlanningDirUnet / mPlanningConfig.timeStampUnet;
    const auto resultDirSR   = rootResultDir / mPlanningDirSR   / mPlanningConfig.timeStampSR;

    LOG_LINE << "evaluate planning in "
      << resultDirGT.string() << "\n"
      << resultDirUnet.string() << "\n"
      << resultDirSR.string();

    std::vector<int> ids;
    std::copy(mSubset1.begin(), mSubset1.end(), std::back_inserter(ids));
    std::copy(mSubset2.begin(), mSubset2.end(), std::back_inserter(ids));

    // prepare some types
    enum EnType
    {
      enUnet = 0,
      enRegularized,
      enGT,
    };

    auto globalStats = CombinedStatistics(ids.size());

    std::vector<int> ks;
    if (mComputeUnetPlanning)
    {
      ks.push_back(enUnet);
      LOG_LINE << "  Selected: compute Unet Planning Results";
    }
    if (mComputePasmPlanning)
    {
      ks.push_back(enRegularized);
      LOG_LINE << "  Selected: compute Pasm Planning Results";
    }
    if (!ks.empty())
      ks.push_back(enGT);
    for (auto k : ks)
    {
      LOG_LINE << "  set data base path to: " << obstacleDirGT.string();
      auto sceneDirBase = k == enUnet ? resultDirUnet : k == enRegularized ? resultDirSR : resultDirGT;
      // first compute the success rate
      for (auto id : ids)
      {
        const auto patientDir = (boost::format("Patient_%02d") % id).str();
        const auto sceneStem  = "scene_vis";
        auto sceneDirGt   = resultDirGT  / patientDir / sceneStem;
        auto sceneDirAuto = sceneDirBase / patientDir / sceneStem;
        const auto idx = id - 1;
        auto& n_Paths = k == enUnet ? globalStats.nUnet[idx] : k == enRegularized ? globalStats.nPasm[idx] : globalStats.nGT[idx];
        // check if the first for letters of the stem match "Path"
        auto is_path = [&] (const fs::path& p)
        {
          auto stem = p.stem().string();
          if (stem.size() < 4)
            return false;
          auto substr = stem.substr(0, 4);
          if (substr == "Path")
            return true;
          else
            return false;
        };
        n_Paths = std::count_if(fs::directory_iterator(sceneDirAuto), fs::directory_iterator(), [&] (const fs::path& p) { return is_path(p); });
      }
    }
    for (auto k : ks)
    {
      auto sceneDirBase = k == enUnet ? resultDirUnet : k == enRegularized ? resultDirSR : resultDirGT;
      // next compute true distances to GT
      LOG_LINE << "  evaluating scenes on ground truth";
      for (auto id : ids)
      {
        // prepare some stuff
        AppModels models;
        ApplicationModel& app  = *models.pAppModel;
        PlanningModel&    plan = *models.pPlanningModel;
        SelectionModel&   select = *models.pSelectionModel;

        const auto patientDir = (boost::format("Patient_%02d") % id).str();
        auto sceneDirGt   = resultDirGT  / patientDir;
        auto sceneDirAuto = sceneDirBase / patientDir;
        // load scene
        auto pScene = app.getScene();
        pScene->setLocalBasePath(obstacleDirGT.string());
        const auto sceneFile = sceneDirAuto / "scene_eval.mukscene";
        app.loadScene(sceneFile.string());
        // configure interpolator
        const auto& c     = mPlanningConfig;
        const auto& canal = c.accessCanal;
        LOG_LINE << "  evaluating access: " << canal;
        plan.setInterpolator(c.interpolator);
        plan.setActivePathCollection(canal); // necessary for SelectionModel
        plan.configurePlanning(canal);
        globalStats.dMin   = pScene->getPathCollection(canal).getProblemDefinition()->getSafetyDist();
        globalStats.radius = pScene->getPathCollection(canal).getProblemDefinition()->getRadius();
        auto* inter   = pScene->getInterpolator();
        for (const auto& pair : c.interpolatorParams)
          inter->setProperty(pair.first, pair.second);
        // compute stats
        const auto& paths = pScene->getPathCollection(canal).getPaths();
        if ( ! paths.empty())
        {
          LOG_LINE << "  found " << paths.size() << " paths";
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
          const auto& dists = select.getDistances();
          LOG << "  distances:";
          for (auto d : dists)
            LOG << " " << d;
          LOG_LINE;
          const auto idx = id - 1;
          auto& dists_auto = k == enUnet ? globalStats.minDistsUnet[idx] : k == enRegularized ? globalStats.minDistsPasm[idx] : globalStats.minDistsGt[idx];
          dists_auto = dists;
        }
      }
    }
    for (auto k : ks)
    {
      auto sceneDirBase = k == enUnet ? resultDirUnet : k == enRegularized ? resultDirSR : resultDirGT;
      LOG_LINE << "  write results to " << sceneDirBase.string();
      globalStats.write(k, sceneDirBase);
    }
    // now evaluate
    if (mWritePlanningTable)
    {
      for (int k(0); k<3; ++k)
      {
        auto sceneDirBase = k == enUnet ? resultDirUnet : k == enRegularized ? resultDirSR : resultDirGT;
        LOG_LINE << "compute global planning results in: " << sceneDirBase;
        globalStats.read(k, sceneDirBase);
        globalStats.writeFinal(k, sceneDirBase);
      }
    }
  }
}
}