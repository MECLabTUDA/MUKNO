#include "private/muk.pch"
#include "private/program_functions.h"
#include "private/program_options.h"
#include "Experiment.h"

#include "MukCommon/InterpolatorFactory.h"
#include "MukCommon/MukException.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/MukObstacle.h"

#include "mukImaging/muk_imaging_tools.h"

#include "MukAppModels/AlgorithmModel.h"
#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/SelectionModel.h"

#include "MukEvaluation/statistics.h"
#include "MukEvaluation/CollisionDetectionhandler.h"

#include "MukQt/QuickPathAnalysisWindow.h"

#include <gstd/XmlDocument.h>
#include <gstd/XmlNode.h>

#include <itkImageFileReader.h>
#include <itkImageToHistogramFilter.h>

#include <vtkPolyData.h>
#include <vtkPolyDataWriter.h>

#include <boost/filesystem.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/tokenizer.hpp>

#pragma warning( disable : 4996 ) // localtime unsafe

namespace
{
  using namespace gris;
  using namespace gris::muk;
  namespace fs = boost::filesystem;
  
  std::vector<std::string> listDataSets = { "P01","P02","P03","P04","P06","P07","P09","P10","P11","P14","P15","P16","P17","P19","P21","P25","P28","P29","P33","P34","P37","P40","P41","P42" };
}

namespace gris
{
namespace muk
{

  /** \brief Counter intuitive: data set 1 starts with the second patient
  */
  std::vector<std::string> getDataSet1()
  {
    std::vector<std::string> result;
    for (int i(1); i < listDataSets.size(); i+=2)
      result.push_back(listDataSets[i]);
    return result;
  }

  /** \brief Counter intuitive: data set 2 starts with the first patient
  */
  std::vector<std::string> getDataSet2()
  {
    std::vector<std::string> result;
    for (int i(0); i < listDataSets.size(); i+=2)
      result.push_back(listDataSets[i]);
    return result;
  }

  /**
  */
  void evalDistances(MukScene& scene, SelectionModel& select, const std::string& canal, int idx, std::ofstream& ofs)
  {
    const auto& coll = scene.getPathCollection(canal);
    
    select.loadPath(idx);
    auto res = select.getIndividual(idx);
    {
      // compute closest point to each risk structure
      std::vector<QuickPathAnalysisWindow::ObstaclePeak> peaks;
      {
        auto keys = scene.getObstacleKeys();
        CollisionDetectionHandler handler;
        handler.initialize(scene.getCollisionDetector().get());
        for (const auto& key : keys)
        {
          if (!scene.getObstacle(key)->getActive())
            continue;
          handler.setupForObstacle(key);
          const MukPath& path = *select.getLoadedPath();
          auto pair = handler.distance(path, coll.getProblemDefinition()->getRadius());
          peaks.push_back(std::make_tuple(key, pair.first, pair.second));
        }
      }
      ofs << "minimal dist\n"    << res.distance.second << std::endl;
      ofs << "length\n"          << res.length.second   << std::endl;
      for (const auto& tuple : peaks)
        ofs << std::get<0>(tuple) << "\n" << std::get<2>(tuple) << std::endl;
    }
  }

  /**
  */
  void createPatientDataFile()
  {
    auto fn = "../resources/MukApplications/Conf_MICCAI2019_OR20/PatientData.mukcfg";
    auto doc = XmlDocument::create("Mukno");
    auto& root = doc->getRoot();
    root.addChild("ImageRootDir").setValue("M:/MUKNO2/Rohdaten/data/preoperative/general/");
    root.addChild("SceneRootDir").setValue("../data/scenes/Conf_MICCAI2019_OR20/");
    auto ndPatients = root.addChild("Patients");
    for (size_t i(1); i<=42; ++i)
    {
      auto& node = ndPatients.addChild("Patient");
      auto id = (boost::format("P%02d") % i).str();
      node.addChild("ID").setValue(id.c_str());
      auto ct = id + "_CT.mhd";
      node.addChild("CT-Image").setValue(ct.c_str());
      auto gt = id + "_GroundTruth.mhd";
      node.addChild("GT-Image").setValue(gt.c_str());
      auto scene = id + "_baseScenarios.mukscene";
      node.addChild("SceneFile").setValue(scene.c_str());
    }
    XmlDocument::save(fn, *doc);
  }

  /**
  */
  void preprocess(const ProgramInput& input, const AlgInfo& alginfo)
  {
    PreprocessingAlgorithm algo (alginfo);
    algo.initialize();
    algo.run();
  }

  /**
  */
  void extractObstacles(const ProgramInput& input)
  {
    LOG_LINE << " ===== Extracting Obstacles ==== ";
    const auto& outDir = input.cochleaData.outputRootDir;
    const auto dir1 = fs::path(outDir) / "AutomatedPASM_PASM1";
    const auto dir2 = fs::path(outDir) / "AutomatedPASM_PASM2";
    fs::path latest1, latest2;
    for (auto iter = fs::directory_iterator(dir1); iter != fs::directory_iterator(); ++iter)
    {
      if (fs::is_directory(iter->path()))
        latest1 = iter->path();
    }
    for (auto iter = fs::directory_iterator(dir2); iter != fs::directory_iterator(); ++iter)
    {
      if (fs::is_directory(iter->path()))
        latest2 = iter->path();
    }
    LOG_LINE << "   Extracting Obstacles from PASM 1";
    for (auto iter = fs::directory_iterator(latest1); iter != fs::directory_iterator(); ++iter)
    {
      if ( ! fs::is_directory(iter->path()))
        continue;
      const auto stem = iter->path().stem().string(); // PXX
      LOG_LINE << "      Patient " << stem;
      for (auto iterVtk = fs::directory_iterator(iter->path()); iterVtk != fs::directory_iterator(); ++iterVtk)
      {
        if (iterVtk->path().extension().string() != ".vtk")
          continue;
        /*const auto fnNew = outDir / 
        fs::copy_file(iterVtk->path());*/      }
    }
  }
  
  /** \brief Saves the passed evaluation in the config file
    
     some idiotic logic to differentiate between preprocessing algorithms and experiments
  */
  void saveEvaluationDate(const std::string& file, EnExperimentType en, const std::string& date, const std::string& alias)
  {
    if ( ! fs::is_regular_file(file))
    {
      throw MUK_EXCEPTION("config file does not exist", file.c_str());
    }
    // validate
    auto pDoc = XmlDocument::read(file.c_str());
    const auto rootNode = pDoc->getRoot().getChild("Conf_MICCAI2019_OR20");
    XmlNode node;
    if (en == enPreprocessingAlgo)
    {
      auto ndsAlgorithm = rootNode.getChild("Preprocessing").getChildren();
      for (const auto& nd : ndsAlgorithm)
      {
        if ( alias == nd.getChild("Alias").getValue())
        {
          node = nd;
          break;
        }
      }
    }
    else
    {
      node = rootNode.getChild("Experiments").getChild(alias);
    }
    node.getChild("LastEvaluationDate").setValue(date.c_str());
    XmlDocument::save(file.c_str(), *pDoc);
  }

  /** \brief Same as saveEvauationDate, but specialized for algorithms within an experiment
  */
  void saveEvaluationDateAlgo(const std::string& file, EnExperimentType en, const std::string& date, const std::string& alias)
  {
    if ( ! fs::is_regular_file(file))
    {
      throw MUK_EXCEPTION("config file does not exist", file.c_str());
    }
    // validate
    auto pDoc = XmlDocument::read(file.c_str());
    const auto rootNode = pDoc->getRoot().getChild("Conf_MICCAI2019_OR20");
    XmlNode node;
    {
      auto baseNode = rootNode.getChild("Experiments").getChild(ScenarioNames[en]);
      auto ndsAlgorithm = baseNode.getChild("Algorithms").getChildren();
      for (const auto& nd : ndsAlgorithm)
      {
        if ( alias == nd.getChild("Alias").getValue())
        {
          node = nd;
          break;
        }
      }
    }
    node.getChild("LastEvaluationDate").setValue(date.c_str());
    XmlDocument::save(file.c_str(), *pDoc);
  }

  /**
  */
  void evaluateStatistics()
  {
    const auto root = fs::path("M:/MUKNO2/Rohdaten/data/preoperative/general");
    auto list1 = getDataSet1();
    auto list2 = getDataSet2();
    std::copy(list2.begin(), list2.end(), std::back_inserter(list1));
    std::sort(list1.begin(), list2.end());

    // output
    std::vector<std::array<long long,11>> hists;

    for (const auto& id : list1)
    {
      hists.push_back(std::array<long long,11>());
      LOG_LINE << "examining " << id;
      const auto file = root / id / (id + "_GroundTruth.mhd" );
      auto reader = make_itk<itk::ImageFileReader<ImageInt3D>>();
      reader->SetFileName(file.string());
      reader->Update();
      
      // compute maximum
      using HistFilter = itk::Statistics::ImageToHistogramFilter<ImageInt3D> ;
      auto histFilter = make_itk<HistFilter>();
      {
        histFilter->SetInput(reader->GetOutput());
        const unsigned int MeasurementVectorSize = 1;   // Grayscale
        const unsigned int binsPerDimension      = 256; // number of bins
        HistFilter::HistogramType::MeasurementVectorType lowerBound(MeasurementVectorSize); // important in generic cases or if automated computation of min/max is turned off?
        HistFilter::HistogramType::MeasurementVectorType upperBound(MeasurementVectorSize);
        lowerBound.Fill(0);
        upperBound.Fill(256);
        HistFilter::HistogramType::SizeType size(MeasurementVectorSize);
        size.Fill(binsPerDimension);
        histFilter->SetMarginalScale(1);
        histFilter->SetHistogramBinMinimum(lowerBound);
        histFilter->SetHistogramBinMaximum(upperBound);
        histFilter->SetHistogramSize(size);
        histFilter->SetAutoMinimumMaximum(false); // this is ridiculously important. if true, the filter computes minimum and maximum and devides the bin values from this min and max
        histFilter->Update();
      }
      auto histogram = histFilter->GetOutput();
      auto& h = hists.back();
      for (int i(0); i<=10; ++i)
      {
        h[i] = histogram->GetFrequency(i);
      }
    }

    auto dir = fs::path("../results/MukApplications/Conf_MICCAI2019_OR20/label_statistics/");
    if (!fs::is_directory(dir))
      fs::create_directories(dir);

    std::ofstream ofs((dir / "mean_frequencies_all.txt").string());
    {
      std::array<double, 11> meanFrequencies;
      meanFrequencies.fill(0.0);
      for (int i(0); i<hists.size(); ++i) // for each file
      {
        const auto N = std::accumulate(hists[i].begin(), hists[i].end(), 0ll);
        for (int j(0); j < hists[i].size(); ++j)
          meanFrequencies[j] += (1.0*hists[i][j]) / N;
      }
      const std::vector<std::string> labels = { "empty", "ICA", "JV", "FN", "Cochlea", "Chorda", "Oss", "SSC", "IAC", "VA", "EAC"};
      const auto M = meanFrequencies.size();
      for (int i(0); i<M; ++i)
      {
        ofs << labels[i] << " " << meanFrequencies[i] / hists.size()  << std::endl;
      }
    }
    std::ofstream ofsl((dir / "mean_frequencies_labels.txt").string());
    {
      std::array<double, 10> meanFrequencies;
      meanFrequencies.fill(0.0);
      for (int i(0); i<hists.size(); ++i)
      {
        const auto N = std::accumulate(hists[i].begin()+1, hists[i].end(), 0ll);
        for (int j(1); j < hists[i].size(); ++j)
          meanFrequencies[j-1] += (1.0*hists[i][j]) / N;
      }
      const std::vector<std::string> labels = {"ICA", "JV", "FN", "Cochlea", "Chorda", "Oss", "SSC", "IAC", "VA", "EAC"};
      const auto M = meanFrequencies.size();
      for (int i(0); i<M; ++i)
      {
        ofsl << labels[i] << " " << meanFrequencies[i] / hists.size()  << std::endl;
      }
    }
  }
}
}