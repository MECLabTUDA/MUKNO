#include "private/muk.pch"
#include "AppControllers.h"
#include "MenuBarController.h"
#include "PropertyController.h"
#include "VisualizationController.h"

#include "MukCommon/MukScene.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/VisualizationModel.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/VisAbstractObject.h"
#include "MukVisualization/VisBounds.h"
#include "MukVisualization/VisCursor3D.h"
#include "MukVisualization/VisObstacle.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisSE3Trajectory.h"

#include <QColorDialog>

namespace gris
{
  namespace muk
  {
    /**
    */
    VisualizationController::VisualizationController()
      : BaseController()
      , mTimerId(0)
    {
    }

    /**
    */
    void VisualizationController::setupConnections()
    {
      connect(mpControls->mpMenuBarController.get(), &MenuBarController::obstacleAdded,   [&] (const std::string& name) { mpModels->pVisModel->addObstacle(name); });
      connect(mpControls->mpMenuBarController.get(), &MenuBarController::obstacleDeleted, [&] (const std::string& name) { mpModels->pVisModel->deleteObstacle(name); });
    }

    /**
    */
    void VisualizationController::addAbstractObject(std::shared_ptr<VisAbstractObject> pObj)
    {
      mpModels->pVisModel->addAbstractObject(pObj);
      mpControls->mpPropControl->addAbstractObject(pObj->getName());
      mpModels->pVisModel->render();
    }

    /**
    */
    void VisualizationController::deleteAbstractObject(const std::string& name)
    {
      mpModels->pVisModel->deleteAbstractObject(name);
      mpControls->mpPropControl->deleteAbstractObject(name);
      mpModels->pVisModel->render();
    }

    /** adds / updates path idx of current active path collection as an abstract object (VisSE3Trajectory)
    */
    void VisualizationController::addTrajectory(const std::string key, int idx)
    {
      if (key.empty() || idx < 0)
        return;
      auto keys  = mpModels->pVisModel->getAbstractObjectKeys();
      const auto visName = std::string(key).append("_").append(std::to_string(idx));
      std::shared_ptr<VisSE3Trajectory> pObj;
      if (std::any_of(keys.begin(), keys.end(), [&] (const auto& str) {return str == visName; }))
      {
        pObj = std::dynamic_pointer_cast<VisSE3Trajectory>(mpModels->pVisModel->getVisScene()->getObject(visName));
      }
      else
      {
        pObj = std::make_shared<VisSE3Trajectory>(visName);
        mpControls->mpVisControl->addAbstractObject(pObj);
      }
      auto pIn = mpModels->pAppModel->getScene()->getInterpolator();
      const auto poses = pIn->getInterpolatedPoses();
      pObj->setPoses(poses);
      pObj->setVisibility(true);
      mpModels->pVisModel->render();
    }

    /**
    */
    VisCursor3D* VisualizationController::requestCursor3D()
    {
      const std::string name = "Cursor3D";
      auto* pVisScene = mpModels->pVisModel->getVisScene();
      auto pObj = pVisScene->getObject(name);
      if (nullptr == pObj)
      {
        auto pCursor = std::make_shared<VisCursor3D>(name);
        addAbstractObject(pCursor);
        return pCursor.get();
      }
      else
      {
        return dynamic_cast<VisCursor3D*>(pObj.get());
      }
    }

    /**
    */
    void VisualizationController::setCursorPosition(Vec3d pos)
    {
      auto cursor = requestCursor3D();
      cursor->setPosition(pos);
      mpModels->pVisModel->render();
    }

    /**
    */
    void VisualizationController::showBounds()
    {
      auto keys = mpModels->pVisModel->getVisScene()->getAbstractObjectKeys();
      if (std::none_of(keys.begin(), keys.end(), [&] (const auto& str) { return str == "Bounds"; }))
      {
        auto pObj = std::make_shared<VisBounds>();
        addAbstractObject(pObj);
      }
      mpModels->pVisModel->showBounds();
      mpModels->pVisModel->render();
    }

    /** \brief initialize timed event at currently 500 milliseconds
    */
    void VisualizationController::addTimer()
    {
      if (mTimerId != 0)
      {
        removeTimer();
      }
      mTimerId = startTimer(500, Qt::PreciseTimer);
    }

    /** \brief Remove timed event
    */
    void VisualizationController::removeTimer()
    {
      killTimer(mTimerId);
      mTimerId = 0;
    }

    /** \brief Slot for Navigation to call on a change of the internal Running State
    */
    void VisualizationController::updateNavState_Running(const bool pState)
    {
      if (pState) 
        addTimer();
      else 
        removeTimer();
    }

    /** \brief Implement timed Event as single call to this->render()
    */
    void VisualizationController::timerEvent(QTimerEvent* e)
    {
      // update the Robot Position and Orientation (State)
      mpModels->pVisModel->updateRobot();
      // update the 'future trajectory'
      mpModels->pVisModel->updateTrajectory();
      // update the current target orientation state
      mpModels->pVisModel->updateTargetOrientation();
      mpModels->pVisModel->render();
    }

    /** \brief Hides all paths except the one with #idx in collection #name
    */
    void VisualizationController::showOnlyPath(const std::string& name, int idx)
    {
      const auto pColl = mpModels->pVisModel->getVisScene()->getPathCollection(name);
      for (size_t i(0); i<pColl->numberOfPaths(); ++i)
      {
        const auto visible = i == idx;
        pColl->getMukPath(i)->setVisibility(visible);
      }
      mpModels->pVisModel->render();
    }

    /**
    */
    void VisualizationController::defaultColorizeObstacles()
    {
      auto pScene = mpModels->pVisModel->getVisScene();
      auto keys = pScene->getObstacleKeys();

      auto colorize1 = [&] (const std::string& obstacle, const std::string& key, const Vec3d& color)
        {
          auto obs = pScene->getObstacle(obstacle);
          std::string tmp;
          std::transform(obs->getName().begin(), obs->getName().end(), back_inserter(tmp), ::tolower);
          if (tmp.find(key) != std::string::npos)
          {
            obs->setDefaultColor(color);
            obs->setColors(color);
            return true;
          }
          return false;
        };
      auto colorize2 = [&] (const std::string& obstacle, const std::string& key, double color[3])
      {
        auto obs = pScene->getObstacle(obstacle);
        std::string tmp;
        std::transform(obs->getName().begin(), obs->getName().end(), back_inserter(tmp), ::tolower);
        if (tmp.find(key) != std::string::npos)
        {
          obs->setDefaultColor(color);
          obs->setColors(color);
          return true;
        }
        return false;
      };

      for (const auto& key : keys)
      {
        if (colorize2(key, "carotid", Colors::Red))
          continue;
        if (colorize2(key, "ica", Colors::Red))
          continue;
        if (colorize2(key, "jugular", Colors::Blue))
          continue;
        if (colorize1(key, "external", Vec3d(170.0/255, 170.0/255, 127.0/255)))
          continue;
        if (colorize1(key, "eac", Vec3d(170.0/255, 170.0/255, 127.0/255)))
          continue;
        if (colorize2(key, "internalau", Colors::Magenta))
          continue;
        if (colorize2(key, "iac", Colors::Magenta))
          continue;

        if (colorize2(key, "facial", Colors::LightYellow))
          continue;
        if (colorize1(key, "brain", Vec3d(2.0/3.0, 2.0/3.0, 1.0)))
          continue;
        if (colorize2(key, "chorda", Colors::Cyan))
          continue;
        if (colorize2(key, "cochlea", Colors::Green))
          continue;

        if (colorize1(key, "semi", Vec3d(1.0, 170.0/250, 0.0)))
          continue;
        if (colorize1(key, "laby", Vec3d(1.0, 170.0/250, 0.0)))
          continue;
        if (colorize1(key, "ossic", Vec3d(170.0/255, 0.0, 1.0)))
          continue;
        if (colorize1(key, "skull", Vec3d(1.0, 170.0/255, 127.0/255)))
          continue;

        // SegThor data set
        if (colorize2(key, "aorta", Colors::Yellow))
          continue;
        if (colorize2(key, "heart", Colors::Magenta))
          continue;
        if (colorize2(key, "esophagus", Colors::Red))
          continue;
        if (colorize2(key, "trachea", Colors::Blue))
          continue;
      }
      mpModels->pVisModel->render();
    }

    /** \brief shows all paths of #PathCollection name
    */
    void VisualizationController::showPaths()
    {
      const auto name = mpModels->pPlanningModel->getActivePathCollection();
      auto vis = mpModels->pVisModel->getVisScene();
      auto col = vis->getPathCollection(name);
      const auto N = col->numberOfPaths();
      for (size_t i(0); i < N; ++i)
        col->getMukPath(i)->setVisibility(true);
      mpModels->pVisModel->render();
    }

    /** \brief hides all paths of #PathCollection name
    */
    void VisualizationController::hidePaths()
    {
      const auto name = mpModels->pPlanningModel->getActivePathCollection();
      auto vis = mpModels->pVisModel->getVisScene();
      auto col = vis->getPathCollection(name);
      const auto N = col->numberOfPaths();
      for (size_t i(0); i < N; ++i)
        col->getMukPath(i)->setVisibility(false);
      mpModels->pVisModel->render();
    }

    /** \brief lets user chose a common color for all paths of #PathCollection name
    */
    void VisualizationController::colorizePaths()
    {
      const auto name = mpModels->pPlanningModel->getActivePathCollection();
      auto vis = mpModels->pVisModel->getVisScene();
      auto col = vis->getPathCollection(name);

      auto dialog = std::make_unique<QColorDialog>();
      dialog->exec();
      if (0 == dialog->result()) // cancel-button = 0, "x"-button = 0, ok-button = 1
        return;
      QColor qcolor = dialog->selectedColor();
      int r, g, b;
      qcolor.getRgb(&r, &g, &b);
      const auto color = Vec3d(r,g,b) / 255.0;
      
      const auto N = col->numberOfPaths();
      for (size_t i(0); i < N; ++i)
        col->getMukPath(i)->setColors(color);
      mpModels->pVisModel->render();
    }
}
}