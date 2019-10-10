#include "private/muk.pch"
#include "private/SurfaceCutter.h"

#include "AppControllers.h"
#include "InteractionController.h"
#include "PropertyController.h"
#include "VisualizationController.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/ProblemDefinitionModel.h"
#include "MukAppModels/VisualizationModel.h"

#include "MukCommon/MukObstacle.h"
#include "MukCommon/vtk_tools.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/VisCursor3D.h"
#include "MukVisualization/VisObstacle.h"
#include "MukVisualization/VisScene.h"

#include "MukQt/MukQRightClickMenu.h"

#include <vtkActor.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphere.h>
#include <vtkTriangle.h>

#include <qinputdialog.h>

namespace
{
  using namespace gris::muk;
  using gris::Vec3d;
  std::string closestObstacle(ICollisionDetector& obj, const Vec3d& p);

  vtkSmartPointer<vtkPolyData> cleanClipped(vtkPolyData*);
}

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(SurfaceCutter);

  /**
  */
  SurfaceCutter::SurfaceCutter()
    : IMukInteraction()
    , mRadius(1.0)
    , mRadiusStepSize(0.5)
    , mpPickedData (nullptr)
  {
    mpClipFunction = make_vtk<vtkSphere>();
    mpFilter       = make_vtk<vtkClipPolyData>();
    mpFilter->SetGenerateClippedOutput(true);
    mpFilter->SetClipFunction(mpClipFunction);
    mpFilter->SetInsideOut(true);
  }

  /**
  */
  SurfaceCutter::~SurfaceCutter()
  {
  }

  /**
  */
  void SurfaceCutter::OnLeftButtonDown()
  {
    if (this->Interactor->GetControlKey()) // strg pressed
    {
      pick();
      cutRegion();
    }
    else
    {
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }
  }

  /**
  */
  void SurfaceCutter::OnMouseMove()
  {
    vtkInteractorStyleTrackballCamera::OnMouseMove();
  }

  /**
  */
  void SurfaceCutter::OnRightButtonDown()
  {
    vtkInteractorStyleTrackballCamera::OnRightButtonDown();
  }

  /**
  */
  void SurfaceCutter::OnRightButtonUp()
  {
    vtkInteractorStyleTrackballCamera::OnRightButtonUp();

    auto pMenu = std::make_unique<MukQRightClickMenu>();
    pMenu->addHandle("Create Obstacle", std::bind(&SurfaceCutter::accept, this));
    pMenu->addHandle("Discard", std::bind(&SurfaceCutter::discard, this));
    pMenu->exec();
  }

  /**
  */
  void SurfaceCutter::OnMouseWheelForward()
  {
    if (this->Interactor->GetControlKey() && mpPickedData) // strg pressed
    {
      mRadius += mRadiusStepSize;
      cutRegion();
    }
    else
    {
      vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
    }
  }

  /**
  */
  void SurfaceCutter::OnMouseWheelBackward()
  {
    if (this->Interactor->GetControlKey() && mpPickedData) // strg pressed
    {
      mRadius -= mRadiusStepSize;
      cutRegion();
    }
    else
    {
      vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
    }
  }

  /**
  */
  void SurfaceCutter::OnKeyPress()
  {
  }
  
  /** \brief sets up the interpolator and synchronizes the controls

  old region data in the manipulator should not be modified, so unload the index
  create a temporal dummy that could be added as start or goal region
  connect the cursor to the DirectionWidget to manipulater the atm uniform direction of the region
  */
  void SurfaceCutter::initialize()
  {
    mpNewRegion = std::make_shared<VisAbstractObject>("TemporalObstacle");
    mpNewRegion->setDefaultColor(Colors::Blue);
    mpNewRegion->setColors(mpNewRegion->getDefaultColor());
    mpControls->mpVisControl->addAbstractObject(mpNewRegion);
    mpCursor = mpControls->mpVisControl->requestCursor3D();
    mpCursor->setArrowVisible(false);
  }

  /**
  */
  void SurfaceCutter::accept()
  {
    auto newName = getNewName();
    if (newName.empty())
    {
      discard();
      return;
    }
    // adapt original obstacle (remove clipped data), this may throw
    adaptPickedObstacle();
    // create new Obstacle
    auto pObs = std::make_shared<MukObstacle>();
    {
      auto pData = cleanClipped(mpNewRegion->getData());
      pObs->setData(pData);
      pObs->setName(newName);
      pObs->setActive(false);
    }
    mpModels->pAppModel->addObstacle(pObs);
    mpModels->pVisModel->addObstacle(pObs->getName());
    finishInteraction();
  }

  /**
  */
  void SurfaceCutter::discard()
  {
    finishInteraction();
  }

  // ------ private functions
  /**
  */
  void SurfaceCutter::adaptPickedObstacle()
  {
    // calculate name of picked obstacle, this is a work around to get the original obstacle
    auto* pDetect = mpModels->pAppModel->getScene()->getCollisionDetector().get();
    auto pickedObstacleName = closestObstacle(*pDetect, mpCursor->getPosition());
    auto pObs = mpModels->pAppModel->getScene()->getObstacle(pickedObstacleName);
    {
      auto pData = cleanClipped(mpFilter->GetClippedOutput());
      pObs->setData(pData);
    }
    mpModels->pVisModel->getVisScene()->getObstacle(pickedObstacleName)->setData(pObs->getData());
    // adapt collision detector
    bool active = pDetect->isActive(pickedObstacleName);
    pDetect->removeObstacle(pickedObstacleName);
    pDetect->addObstacle(pickedObstacleName, pObs->getData());
    pDetect->setActive(pickedObstacleName, active);
  }

  /**
  */
  void SurfaceCutter::pick()
  {
    if ( ! this->Interactor )
      return;
    auto pPicker = make_vtk<vtkPropPicker>();
    pPicker->Pick(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1], 0, mpRenderer);
    double picked[3];
    pPicker->GetPickPosition(picked);
    mpCursor->setPosition(Vec3d(picked));
    auto* pActor = pPicker->GetActor();
    if (nullptr == pActor)
    {
      mpPickedData = nullptr;
    }
    else
    {
      mpPickedData = dynamic_cast<vtkPolyData*>(pActor->GetMapper()->GetInputDataObject(0, 0));
    }
    mpFilter->SetInputData(mpPickedData);
    mpFilter->Modified();
  }

  /**
  */
  void SurfaceCutter::cutRegion()
  {
    if ( ! mpPickedData)
      return;
    mpClipFunction->SetCenter(mpCursor->getPosition().x(), mpCursor->getPosition().y(), mpCursor->getPosition().z());
    mpClipFunction->SetRadius(mRadius);
    mpClipFunction->Modified();
    mpFilter->Update();
    // do this already here only for visualization purposes
    mpNewRegion->setData(mpFilter->GetOutput());
    mpModels->pVisModel->render();
  }

  /**
  */
  void SurfaceCutter::finishInteraction()
  {
    mpControls->mpVisControl->deleteAbstractObject("TemporalObstacle");
    mpControls->mpPropControl->reloadProperty();
    mpModels->pVisModel->render();
    mpControls->mpInteract->setInteraction("default");
  }

  /**
  */
  std::string SurfaceCutter::getNewName()
  {
    bool ok;
    auto pDialog = std::make_unique<QInputDialog>();
    bool keepAsking = true;
    std::string ret;
    do
    {
      QString result = pDialog->getText(0, "Create new Obstacle", "Obstacle name:", QLineEdit::Normal,
        "CutOut", &ok);
      if (ok)
      {
        auto keys = mpModels->pAppModel->getScene()->getObstacleKeys();
        if (result.isEmpty() || std::any_of(keys.begin(), keys.end(), [&] (const std::string& key) {return key == result.toLocal8Bit().constData(); }))
        {
          LOG_LINE << "Invalid obstacle name. Name is empty or already exists.";
        }
        else
        {
          keepAsking = false;
          ret = result.toLocal8Bit().constData();
        }
      }
      else
      {
        keepAsking = false;
      }
    }
    while(keepAsking);
    return ret;
   }
}
}

namespace
{
  std::string closestObstacle(ICollisionDetector& obj, const Vec3d& p)
  {
    auto keys = obj.getKeys();
    std::vector<std::pair<std::string, bool>> oldParams;
    for (const auto& key : keys) 
      oldParams.push_back(std::make_pair(key, obj.isActive(key)));

    auto l_deactivate = [&] () { for (const auto& key : keys) { obj.setActive(key, false); } };
    std::vector<std::pair<std::string, double>> distances;
    for (const auto& key : keys)
    {
      l_deactivate();
      obj.setActive(key, true);
      obj.rebuild();
      Vec3d nn;
      obj.nearestNeighbor(p, nn);
      distances.push_back(std::make_pair(key, (p-nn).squaredNorm()));
    }
    auto iter = std::min_element(distances.begin(), distances.end(), [&] (const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
    for (const auto& pair : oldParams) 
      obj.setActive(pair.first, pair.second);
    
    return iter->first;
  }
}

namespace
{
  vtkSmartPointer<vtkPolyData> cleanClipped(vtkPolyData* pInput)
  {
    auto pOutput = make_vtk<vtkPolyData>();
    auto pFilter = make_vtk<vtkCleanPolyData>();
    pFilter->SetInputData(pInput);
    pFilter->Update();
    pOutput = pFilter->GetOutput();
    return pOutput;
  }
}