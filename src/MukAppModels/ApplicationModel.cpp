#include "private/muk.pch"
#include "ApplicationModel.h"
//#include "CtVisModel.h"
#include "PlanningModel.h"
#include "MukAppModels/VisualizationModel.h"
#include "LocalEnvironment.h"
//#include "NavigationThread.h"

#include "MukCommon/MukException.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/MukObstacle.h"
#include "MukCommon/Bounds.h"
#include "MukCommon/mukIO.h"
#include "MukCommon/version.h"
#include "MukCommon/geometry.h"
#include "MukCommon/PlannerFactory.h"
#include "MukCommon/InterpolatorFactory.h"
#include "MukCommon/OptimizerFactory.h"
#include "MukCommon/CollisionDetectorKdTree.h"
#include "MukCommon/MukStringToolkit.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/mukItkIO.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/PolyDataHandler.h"
#include "MukVisualization/PolyDataMapperHandler.h"
#include "MukVisualization/VisMukPath.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"


#include "MukQt/MukQToolBar.h"
#include "MukQt/MukQMenuBar.h"
#include "MukQt/TabPlanning.h"
#include "MukQt/muk_qt_tools.h"

#include "gstd/XmlDocument.h"
#include "gstd/XmlNode.h"

#include <QtWidgets/QMessageBox>

#include <itkImageFileReader.h>
#include <itkImageToVTKImageFilter.h>

#include "vtkQuadricDecimation.h"
#include "vtkImageMarchingCubes.h"
#include "vtkPolyDataNormals.h"
#include "vtkPolyDataMapper.h"

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include <algorithm>
#include <functional>

namespace
{
	namespace fs = boost::filesystem;
  bool path_contains_file(fs::path dir, fs::path file);
  fs::path extract_sub_directory(const fs::path& dir, const fs::path& file);
}

namespace gris
{
	namespace muk
	{
		/**
		*/
    ApplicationModel::ApplicationModel()
    : BaseModel()
		, mpScene(std::make_shared<MukScene>())
    , mExecutableDir(".")
    , mWorkingDir(".")
		{
		}

		/**
		*/
  void ApplicationModel::handleException(const MukException& e)
  {
    QMessageBox infobox;
    auto text = boost::format("Exception occured in:\n%s") % e.getFunction();
    infobox.setText(text.str().c_str());
    auto infoText = boost::format("%s\n\n%s") % e.getReason() % e.getInfo();
    LOG_LINE << "MukException\n" << text << "\n" << infoText;
    infobox.setInformativeText(infoText.str().c_str());
    infobox.setIcon(QMessageBox::Warning);
    infobox.setStandardButtons(QMessageBox::Ok);
    infobox.setWindowTitle("Gris-Exception");
    infobox.exec();
  }

  /**
  */
  void ApplicationModel::handleException(const std::exception& e)
  {
    QMessageBox infobox;
    infobox.setText("std::exception catched!");
    infobox.setInformativeText(e.what());
    infobox.setIcon(QMessageBox::Warning);
    infobox.setStandardButtons(QMessageBox::Ok);
    infobox.setWindowTitle("std::exception");
    //infobox.setIconPixmap();
    infobox.exec();
  }

  /**
  */
		void ApplicationModel::loadObstacle(const std::string& filename)
		{
			boost::filesystem::path fn(filename);
			TLOG << "Loading File:\n   '" << fn << "' ...";
			if (!fs::is_regular_file(fn))
			{
				throw MukException(__FUNCTION__, MukException::enFileNotFound, fn.string().c_str());
			}
			std::vector<vtkSmartPointer<vtkPolyData>> obstacles = loadAnyFile(fn.string());
			LOG_LINE << "loading finished.";
			const size_t N = mpScene->getObstacleKeys().size();
			const size_t M = obstacles.size();
			TLOG << "Found " << M << " obstacles. Add obstacles...";
			for (size_t j(0); j < obstacles.size(); ++j)
			{
				std::string key = M == 1 ? fn.stem().string() : (boost::format("%s_%d") % fn.stem().string() % (N + j)).str();
				auto pObj = std::make_shared<MukObstacle>();
				pObj->setName(key);
				if (M == 1)
					pObj->setFileName(filename);
				pObj->setData(obstacles[j]);
				pObj->setActive(true);
				mpScene->insertObstacle(pObj);
			}
			LOG_LINE << "done";
					}

		/**
		*/
		void ApplicationModel::saveObstacle(const std::string& filename, const std::string& name)
		{
			LOG << "saving to " << name << "...";
			const auto obj = mpModels->pAppModel->getScene()->getObstacle(name);
			saveToVtkFile(obj->getData(), filename.c_str());
			LOG << "done.\n";
		}

		/**
		*/
		void ApplicationModel::addObstacle(std::shared_ptr<MukObstacle> pObj)
		{
			mpScene->insertObstacle(pObj);
			//emit obstacleAdded(pObj->getName());
		}

		/**
		*/
		/**
		*/
		void ApplicationModel::reset()
		{
			auto keys = mpScene->getPathKeys();
			for (const auto& key : keys)
			{
				mpModels->pPlanningModel->deletePathCollection(key.c_str());
			}
			keys = mpScene->getObstacleKeys();
			for (const auto& key : keys)
			{
				deleteObstacle(key);
			}
      if (mpModels->pVisModel)
      {
        keys = mpModels->pVisModel->getAbstractObjectKeys();
        for (const auto& key : keys)
        {
          mpModels->pVisModel->deleteAbstractObject(key.c_str());
        }
      }
			mpScene->reset();
		}

		/**
		*/
		void ApplicationModel::deleteObstacle(const std::string& name)
		{
			//emit obstacleDeleted(name);
			mpScene->deleteObstacle(name);
		}
    
		/**
		*/
  void ApplicationModel::saveScene(const std::string& filename_scene) const
		{
      // remove base path from obstacle files
      auto keys = mpScene->getObstacleKeys();
      fs::path basepath = mpModels->pLocal->getRawDataPath();
      for (const auto& key : keys)
      {
        auto pObj = mpScene->getObstacle(key);
        fs::path filename = pObj->getFileName();
        if (path_contains_file(basepath, filename))
        {
          pObj->setFileName( extract_sub_directory(basepath, filename).generic_string() );
        }
      }
      mpScene->save(filename_scene);
		}

		/**
		*/
		void ApplicationModel::loadScene(const std::string& filename)
		{
			reset();
			mpScene->load(filename);
      mpScene->getPlanner()->initialize();
      LOG_LINE << "Loaded scene " << filename;
		}

		/**
		*/
		void ApplicationModel::savePath(const std::string& filename)
		{
			const auto& key  = mpModels->pPlanningModel->getActivePathCollection();
      const int   idx  = mpModels->pPlanningModel->getActivePathIdx();
      auto path = mpModels->pVisModel->getVisScene()->getPathCollection(key)->getMukPath(idx)->asMukPath();
			ofstream ofs(filename);
      ofs << path.getRadius() << endl;
      for (const auto& state : path.getStates())
      {
        ofs << state.coords << " " << state.tangent << endl;
      }
		}

		/**
		*/
		void ApplicationModel::loadPath(const std::string& filename)
		{
      const auto& key  = mpModels->pPlanningModel->getActivePathCollection();
      mpModels->pPlanningModel->loadPath(key, filename);
		}
    
    /**
    */
    void ApplicationModel::setExecutableDir(const std::string & str)
    {
      mExecutableDir = str;
    }

    /**
    */
    const std::string& ApplicationModel::getExecutableDir() const
    {
      return mExecutableDir;
    }

    /**
    */
    void ApplicationModel::setWorkingDir(const std::string & str)
    {
      mWorkingDir = str;
    }

    /**
    */
    const std::string& ApplicationModel::getWorkingDir() const
    {
      return mWorkingDir;
    }
	}
}



namespace
{
  /**
  */
  bool path_contains_file(fs::path dir, fs::path file)
  {
    // If dir ends with "/" and isn't the root directory, then the final
    // component returned by iterators will include "." and will interfere
    // with the std::equal check below, so we strip it before proceeding.
    if (dir.filename() == ".")
      dir.remove_filename();
    // We're also not interested in the file's name.
    assert(file.has_filename());
    file.remove_filename();

    // If dir has more components than file, then file can't possibly
    // reside in dir.
    auto dir_len = std::distance(dir.begin(), dir.end());
    auto file_len = std::distance(file.begin(), file.end());
    if (dir_len > file_len)
      return false;

    // This stops checking when it reaches dir.end(), so it's OK if file
    // has more directory components afterward. They won't be checked.
    return std::equal(dir.begin(), dir.end(), file.begin());
  }

  /** \brief doesn't actually remove subdirectoy, but as many directories as in dir
  */
  fs::path extract_sub_directory(const fs::path& dir, const fs::path& file)
  {
    fs::path result;
    auto file_iter = file.begin();
    auto dir_iter = dir.begin();
    for (; file_iter != file.end() && dir_iter != dir.end(); ++file_iter, ++dir_iter)
    {
      if (*file_iter != *dir_iter)
        break;
    }
    if (dir_iter == dir.end())
      // finished dir-iter first
    {
      while (file_iter != file.end()) { result /= *file_iter; ++file_iter; }
      return result;
    }
    else
      return file;

  }
}
