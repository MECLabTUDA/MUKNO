#include "private/muk.pch"
#include "Experiment.h"
#include "private/program_functions.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include "MukAppModels/AlgorithmModel.h"

#include <itkImageFileWriter.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataWriter.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <iomanip>

namespace
{
  namespace fs = boost::filesystem;


  const std::array<std::string, gris::muk::enImageTypeSize> DataTypeNames = 
  {
    "ImageInt2D",
    "ImageInt3D",
    "ImageFloat3D",
    "GradientImage3D",
    "VtkImage",
    "VtkMesh",
    "VtkPolyData",
  };
}

namespace gris
{
namespace muk
{
  /**
  */
  void Experiment::initialize()
  {
    if (! fs::is_regular_file(mInput.S_PatientDataFile) )
      throw MUK_EXCEPTION("Passed PatientDataFile is not a regular file: ", mInput.S_PatientDataFile.c_str());

    // created date;
    {
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::ostringstream oss;
      oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
      mInput.lastEvalDate = oss.str();
      saveEvaluationDate(mInput.S_AppConfigFile, mExpType, mInput.lastEvalDate, ScenarioNames[mExpType]);
    }
    // created result output directories;
    fs::path outDir;
    outDir = fs::path(mInput.outputDir) / ScenarioNames[mExpType] / mInput.lastEvalDate;
    if ( ! fs::is_directory(outDir) )
      fs::create_directories(outDir);

    // copy the config file into the output directory
    fs::copy_file(mInput.S_AppConfigFile, outDir / fs::path(mInput.S_AppConfigFile).filename());
  }

  /**
  */
  bool Experiment::skipPatient(const PatientData& patient)
  {
    const auto patientID = stoi(patient.id.substr(1,2));
    if (!mInput.skipPatientsUnequal.empty() && std::none_of(mInput.skipPatientsUnequal.begin(), mInput.skipPatientsUnequal.end(), [&](const auto id) { return id == patientID; }))
      return true;
    if ( mInput.skipPatientsLarger >= 0 && patientID > mInput.skipPatientsLarger)
      return true;
    if ( mInput.skipPatientsSmaller >= 0 && patientID < mInput.skipPatientsSmaller)
      return true;
    if (std::any_of(mInput.skipPatients.begin(), mInput.skipPatients.end(), [&] (const auto& str) { return str == patient.id; }))
    {
      return true;
    }
    return false;
  }

  /**
  */
  void Experiment::run()
  {
    LOG_LINE << "===========================";
    LOG_LINE << "run";
    for (const auto& patient : mInput.patientData)
    {
      LOG_LINE << "  --------------------------";
      LOG_LINE << "  Patient " << patient.id;
      if (skipPatient(patient))
        continue;
      try
      {
        runExperiment(patient);
      }
      catch(std::exception& e)
      {
        LOG_LINE << " -> could not run for id " << patient.id << ": " << e.what();
      }
    }
    LOG_LINE << "finished run";
    LOG_LINE << "===========================";
  }

  /**
  */
  void Experiment::eval()
  {
    LOG_LINE << "===========================";
    LOG_LINE << "evaluate";
    for (const auto& patient : mInput.patientData)
    {
      LOG_LINE << "  --------------------------";
      LOG_LINE << "  Patient " << patient.id;
      if (skipPatient(patient))
        continue;
      try
      {
        evalExperiment(patient);
      }
      catch(std::exception& e)
      {
        LOG_LINE << " -> could not evaluate for id " << patient.id << ": " << e.what();
      }
    }
    try
    {
      LOG_LINE << "  --------------------------";
      LOG_LINE << "  Summary";
      evalExperiments();
    }
    catch(std::exception& e)
    {
      LOG_LINE << " -> could not summarize evaluation: " << e.what();
    }
    LOG_LINE << "finished evaluation";
    LOG_LINE << "===========================";
  }

  /**
  */
  PreprocessingAlgorithm::PreprocessingAlgorithm(const AlgInfo& input, EnExperimentType type)
    : Experiment(input, type)
    , mAlgInfo(input)
  {
  }

  /**
  */
  /**
  */
  void PreprocessingAlgorithm::initialize()
  {
    if (mExpType != enPreprocessingAlgo)
    {
      // created date;
      {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        mInput.lastEvalDate = oss.str();
        saveEvaluationDateAlgo(mInput.S_AppConfigFile, mExpType, mInput.lastEvalDate, mAlgInfo.alias);
      }
      // created result output directories;
      fs::path outDir;
      outDir = fs::path(mInput.outputDir) / mAlgInfo.alias / mInput.lastEvalDate;
      if ( ! fs::is_directory(outDir) )
        fs::create_directories(outDir);

      // copy the config file into the output directory
      fs::copy_file(mInput.S_AppConfigFile, outDir / fs::path(mInput.S_AppConfigFile).filename());
    }
  }

  /** \brief Applies an algorithm-graph with the help of the AlgorithModel

    Algorithms always read and/or write something.
    Everything that needs to be read/written contains the patient id in the filename (by convention).

    Example:
    An algorithm that extract the brain would read e.g. from 
    "<some/path/..>/P01_CT.mhd"
    and write to
    "<maybe/some/other/path/..>/P01_Skull.mhd".

    An algorithm has at least one reader and one writer with a property "Filename".
    This function searches for properties "Filename" and values that contain "P01" (again, by convention "template" algorithms use P01 as sample value).
    The parameter #algIDs contains the ids of the algorithms that need to be changed and replace "P01" wiht P<newID>".

    \param filename filename of an algorithm xml file (.alg)
    \param algIDS   ids of the algorithm graph that correspond to an algorithm that needs to be changed.
    \param newID    the new patient ID
  **/
  void PreprocessingAlgorithm::runExperiment(const PatientData& patient)
  {
    using namespace gris::muk;
    LOG_LINE << "   " << mAlgInfo.alias << ": updating id " << patient.id;
    const auto& filename = mAlgInfo.filename;
    const auto& idsIn = mAlgInfo.ids;
    unsigned int newID = std::atoi(patient.id.substr(1,2).c_str());
    {
      AlgorithmModel model;
      model.loadAlgorithm(filename);
      for (const auto id : idsIn)
      {
        std::string templateRefString = (boost::format("P%02d") % 1).str();
        std::string templateNewString = (boost::format("P%02d") % newID).str();
        std::string referenceStringDate = "XXXLatestDate";

        const std::string propName = "Filename";
        if (model.getAlgorithm(id).hasProperty(propName))
        {
          std::string value;
          model.getAlgorithm(id).getProperty(propName, value);
          auto replaceAllValues = [&] (std::string& value, const std::string& reference, const std::string& referenceReplace)
          {
            size_t offset(0);
            size_t pos(0);
            while (true)
            {
              pos = value.find(reference, offset);
              offset = pos+1;
              if (std::string::npos == pos)
              {
                break;
              }
              value.replace(pos, reference.size(), referenceReplace);
            }
          };
          replaceAllValues(value, templateRefString, templateNewString);
          LOG_LINE << "replace " << referenceStringDate << " with " << mAlgInfo.parentAlgorithmEvalDate;
          replaceAllValues(value, referenceStringDate, mAlgInfo.parentAlgorithmEvalDate);
          auto newDir = fs::path(value).parent_path();
          if ( ! fs::is_directory(newDir))
            fs::create_directories(newDir);
          LOG_LINE << "Read from " << value;
          model.getAlgorithm(id).setProperty(propName, value);
        }

        const std::string propNameInitFile = "InitFile";
        if (model.getAlgorithm(id).hasProperty(propNameInitFile))
        {
          std::string value;
          model.getAlgorithm(id).getProperty(propNameInitFile, value);
          size_t offset(0);
          size_t pos(0);
          // find last occurence
          while (true)
          {
            auto posTmp = value.find(templateRefString, offset);
            if (std::string::npos == posTmp)
            {
              break;
            }
            pos = posTmp;
            offset = pos+1;
          }
          // replace last occurence
          value.replace(pos, templateNewString.size(), templateNewString);
          model.getAlgorithm(id).setProperty(propNameInitFile, value);
        }
      }
      model.update();
      // by definition, all "out" algorithms have one output port of type ImageInt3D
      LOG_LINE << "writing output of " << mAlgInfo.alias;
      for (int i(0); i < mAlgInfo.outIds.size(); ++i)
      {
        const auto& id = mAlgInfo.outIds[i];
        LOG_LINE << "  outID   " << id;
        LOG_LINE << "  outType " << ::DataTypeNames[mAlgInfo.outTypes[i]];
        // if only one output, its some P01-thing. If more outputs, take the aliases
        const auto dir = fs::path(mAlgInfo.outputDir) / mAlgInfo.alias / mAlgInfo.lastEvalDate / (boost::format("P%02d") % newID).str();
        fs::path fn;
        if (mAlgInfo.outIds.size() == 1)
        {
          fn = dir / (boost::format("P%02d.mhd") % newID).str();
        }
        else
        {
          switch (mAlgInfo.outTypes[i])
          {
            case enImageInt3D:
            {
              const auto& alias = model.getAlgorithm(id).getAlias();
              if (!alias.empty())
                fn = dir / (alias + ".mhd");
              else
                fn = dir / (patient.id + ".mhd");
              break;
            }
            case enVtkPolyData:
            {
              fn = dir / (model.getAlgorithm(id).getAlias() + ".vtk");
              break;
            }
            default:
            {
              LOG_LINE << "Warning: no support for this file type";
            }
          }
        }

        std::string filename = fn.string();
        LOG_LINE << "  writing to " << filename;
        // make sure directory to write into exists
        if (!fs::is_directory(dir))
          fs::create_directories(dir);
        
        // write depending on type
        switch(mAlgInfo.outTypes[i])
        {
          case enImageInt3D:
          {
            auto* img = static_cast<ImageInt3D*>(model.getAlgorithm(id).getOutput(0));
            auto writer = make_itk<itk::ImageFileWriter<ImageInt3D>>();
            writer->SetFileName(filename.c_str());
            writer->SetInput(img);
            writer->Update();
            break;
          }
          case enVtkPolyData:
          {
            vtkPolyData* mesh = nullptr;
            // quick fix, one of these is true
            if (model.getAlgorithm(id).getOutputType(0) == enVtkPolyData)
              mesh = static_cast<vtkPolyData*>(model.getAlgorithm(id).getOutput(0));
            else if (model.getAlgorithm(id).getOutputType(1) == enVtkPolyData)
              mesh = static_cast<vtkPolyData*>(model.getAlgorithm(id).getOutput(1));
            auto writer = make_vtk<vtkPolyDataWriter>();
            writer->SetFileName(filename.c_str());
            writer->SetInputData(mesh);
            writer->Update();
            break;
          }
        }
      }
    }
  }
}
}