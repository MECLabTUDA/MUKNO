#include "private/muk.pch"
#include "WorldVisualizationModel.h"
#include "private/WorldVisualizationModelImpl.h"

#include "MukVisualization/VisAbstractObject.h"

#include "vtkRenderWindow.h"

namespace gris
{
namespace muk
{
  /**
  */
  WorldVisualizationModel::WorldVisualizationModel()
    : mp(std::make_unique<Impl>())
  {
    declareProperty<double>("ColorWindow"
      , [&] (double val)
          {
            mp->mpAxialSliceWidget->getRenderGroup().setColorWindow(val);
            mp->mpSagittalSliceWidget->getRenderGroup().setColorWindow(val); 
            mp->mpCoronalSliceWidget->getRenderGroup().setColorWindow(val); 
          }
      , [&] () { return mp->mpAxialSliceWidget->getRenderGroup().getColorWindow();  });

    declareProperty<double>("ColorLevel"
      , [&] (double val)
    {
      mp->mpAxialSliceWidget->getRenderGroup().setColorLevel(val);
      mp->mpSagittalSliceWidget->getRenderGroup().setColorLevel(val); 
      mp->mpCoronalSliceWidget->getRenderGroup().setColorLevel(val); 
    }
    , [&] () { return mp->mpAxialSliceWidget->getRenderGroup().getColorLevel();  });
  }

  /**
  */
  WorldVisualizationModel::~WorldVisualizationModel()
  {
  }

  /**
  */
  void WorldVisualizationModel::setAxialSliceWidget(SliceWidget* pWidget)
  {
    mp->mpAxialSliceWidget = pWidget;
    mp->mpAxialGroup = &pWidget->getRenderGroup();
  }
  
  /**
  */
  void WorldVisualizationModel::setSagittalSliceWidget(SliceWidget* pWidget)
  {
    mp->mpSagittalSliceWidget = pWidget;
    mp->mpSagittalGroup = &pWidget->getRenderGroup();
  }

  /**
  */
  void WorldVisualizationModel::setCoronalSliceWidget(SliceWidget* pWidget)
  {
    mp->mpCoronalSliceWidget = pWidget;
    mp->mpCoronalGroup = &pWidget->getRenderGroup();
  }

  /**
  */
  void WorldVisualizationModel::set3DWindow(VtkWindow* pWidget)
  {
    mp->mp3DWindow = pWidget;
  }

  /**
  */
  void WorldVisualizationModel::setCtImage(vtkImageData* pImage)
  {
    mp->setCtImage(pImage);
    // deactivate segmentation if size is unequal
  }

  /**
  */
  void WorldVisualizationModel::setSegmentationImage(vtkImageData* pImage)
  {
    mp->setSegmentationImage(pImage);
  }

  /**
  */
  void WorldVisualizationModel::render()
  {
    mp->mpAxialGroup->getRenderWindow()->Render();
    mp->mpSagittalGroup->getRenderWindow()->Render();
    mp->mp3DWindow->Render();
    mp->mpCoronalGroup->getRenderWindow()->Render();
  }

  /**
  */
  void WorldVisualizationModel::initialize()
  {
    mp->initialize();
  }

  /**
  */
  void WorldVisualizationModel::setSegmentationObject(const std::string& name, vtkSmartPointer<vtkPolyData>& pObj)
  {
    auto* pVisObj = mp->getSegmentationObject(name);
    pVisObj->setData(pObj);
    mp->mp3DWindow->Render();
  }

  /**
  */
  void WorldVisualizationModel::deleteSegmentationObject(const std::string& name)
  {
    auto iter = std::find_if(mp->mAlgorithmOutputs.begin(), mp->mAlgorithmOutputs.end(), [&] (const auto& obj) { return obj->getName() == name; } );
    if (iter != mp->mAlgorithmOutputs.end())
      mp->mAlgorithmOutputs.erase(iter);
    mp->mp3DWindow->Render();
  }

  /**
  */
  size_t WorldVisualizationModel::numberOfAlgorithmOutputs() const
  {
    return mp->mAlgorithmOutputs.size();
  }

  /**
  */
  VisAbstractObject* WorldVisualizationModel::getAlgorithmOutput(size_t i) const
  {
    if (i >= mp->mAlgorithmOutputs.size())
      return nullptr;
    else
      return mp->mAlgorithmOutputs[i].get();
  }

  /**
  */
  VisAbstractObject* WorldVisualizationModel::getAlgorithmOutput(const std::string& name) const
  {
    auto iter = std::find_if(mp->mAlgorithmOutputs.begin(), mp->mAlgorithmOutputs.end(), [&] (const auto& pObj)
    {
      return name == pObj->getName();
    });
    if (iter == mp->mAlgorithmOutputs.end())
      return nullptr;
    else
      return iter->get();
  }

  /**
  */
  vtkImageData* WorldVisualizationModel::getCtImage() const
  {
    if (mp->mpAxialGroup)
      return mp->mpAxialGroup->getImage();
    else
      return nullptr;
  }
}
}

