#include "private/muk.pch"
#include "private/PropertyDetails.h"

#include "AppControllers.h"
#include "MenuBarController.h"
#include "PlanningController.h"
#include "PropertyController.h"
#include "ToolbarController.h"
#include "VisualizationController.h"

#include "MukAppModels/ApplicationModel.h"
#include "MukAppModels/LocalEnvironment.h"
#include "MukAppModels/PlanningModel.h"
#include "MukAppModels/PropertyModel.h"
#include "MukAppModels/VisualizationModel.h"
#include "MukAppModels/WorldVisualizationModel.h"
#include "NavigationThread.h"

#include "MukCommon/MukScene.h"
#include "MukCommon/MukObstacle.h"
#include "MukCommon/MukPathGraph.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/vtk_tools.h"

#include "MukAlgorithms/AlgorithmWrapper.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/VisObstacle.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"
#include "MukVisualization/VisMukPathGraph.h"

#include "MukQt/muk_qt_tools.h"
#include "MukQt/MuknoPlannerMainWindow.h"
#include "MukQt/SceneWidget.h"

#include <vtkPolyData.h>
#include <vtkPolyDataWriter.h>

#include <qtreewidget.h>
#include <qfiledialog.h>

#include <boost/filesystem.hpp>

namespace
{
  // workaround for still incomplete DynamicProperty-class

}

namespace gris
{
namespace muk
{
  /**
  */
  PropertyController::PropertyController()
    : mpSceneWidget(nullptr)
    , mpPropertyWidget(nullptr)
    , mCachedType(EnPropertyType::enSize_Property_Type)
  {
    std::vector<std::string> types;
    for (int i(0); i<VisMukPath::EnTopology::N_Topologies; ++i)
    {
      types.push_back(VisMukPath::s_topologyStr[i]);
    }
    mStringSelections["Topology"] = types;
    mStringSelections["DefaultPathTopology"] = types;
    mStringSelections["InterpolationType"] = IInterpolator::getInterpolationTypes();
    for (size_t i(0); i<DisplayTypeNames.size(); ++i)
      mStringSelections["DisplayType"].push_back(DisplayTypeNames[i]);
  }

  /**
  */
  void PropertyController::setupConnections()
  {
    connect(mpControls->mpMenuBarController.get(), &MenuBarController::obstacleDeleted, this, SELECT<const std::string&>::OVERLOAD_OF(&PropertyController::deleteObstacle));

    connect(mpSceneWidget,    &QTreeWidget::itemClicked, this, SELECT<QTreeWidgetItem*, int>::OVERLOAD_OF(&PropertyController::showProperty));
    connect(mpPropertyWidget, &PropertyWidget::propertyChanged, this, &PropertyController::updateProperty);    

    connect(mpSceneWidget, &SceneWidget::saveObjectClicked, this, &PropertyController::saveObject);
    connect(mpSceneWidget, &SceneWidget::deleteObstacleClicked, this, SELECT<void>::OVERLOAD_OF(&PropertyController::deleteObstacle));
    connect(mpSceneWidget, &SceneWidget::deleteAbstractObjectClicked, this, SELECT<void>::OVERLOAD_OF(&PropertyController::deleteAbstractObject));

    connect(mpSceneWidget, &SceneWidget::showPaths, [&] () { mpControls->mpVisControl->showPaths(); });
    connect(mpSceneWidget, &SceneWidget::hidePaths, [&] () { mpControls->mpVisControl->hidePaths(); });
    connect(mpSceneWidget, &SceneWidget::colorize,  [&] () { mpControls->mpVisControl->colorizePaths(); });

    connect(mpSceneWidget, &SceneWidget::showOrientationPathClicked, this, &PropertyController::showTrajectory);
    connect(mpSceneWidget, &SceneWidget::showPlannerGraphClicked, this, &PropertyController::extractPlanningGraph);

    connect(mpSceneWidget, &SceneWidget::defaultColorizeObstaclesClicked, [&] () { mpControls->mpVisControl->defaultColorizeObstacles(); });
    connect(mpSceneWidget, &SceneWidget::textDropped, this, &PropertyController::evaluateDroppedText);

    connect(mpSceneWidget, &SceneWidget::showBoundsClicked, this, &PropertyController::showBounds);
    connect(mpSceneWidget, &SceneWidget::setDefaultBoundsClicked, [&] () { mpControls->mpPlanningController->setDefaultBounds(); });
    connect(mpSceneWidget, &SceneWidget::minimizeBoundsClicked,   [&] () { mpControls->mpPlanningController->minimizeBounds(); });
    connect(mpSceneWidget, &SceneWidget::addObjectBoundsClicked,  [&] () { mpControls->mpPlanningController->addObjectBounds(); });

    connect(mpControls->mpNaviThread.get(), &NavigationThread::navigatorChanged, this, &PropertyController::updateNavigator);
  };

  /**
  */
  void PropertyController::initSceneWidget()
  {
    mpSceneWidget->clear();
    // init basic top level items
    QTreeWidgetItem* item;
    for (int i(0); i<enSize_Property_Type; ++i)
    {
      item = new QTreeWidgetItem();
      item->setText(0, PropertyTypeNames[i]);
      item->setFont(0, QFont("", 9, QFont::Bold));
      mpSceneWidget->addTopLevelItem(item);
    }

    // insert models
    {
      auto* item = mpSceneWidget->topLevelItem(enAppModels);
      {
        auto* modelItem = new QTreeWidgetItem();
        modelItem->setText(0, WorldVisualizationModel::s_name());
        modelItem->setFont(0, QFont("", 9, QFont::Bold));
        item->addChild(modelItem);
        modelItem = new QTreeWidgetItem();
        modelItem->setText(0, LocalEnvironment::s_name());
        modelItem->setFont(0, QFont("", 9, QFont::Bold));
        item->addChild(modelItem);
        modelItem = new QTreeWidgetItem();
        modelItem->setText(0, VisualizationModel::s_name());
        modelItem->setFont(0, QFont("", 9, QFont::Bold));
        item->addChild(modelItem);
      }
    }

    // insert rest
    auto* pScene = mpModels->pAppModel->getScene().get();
    setPlanner(pScene->getPlanner()->name());
    setPruner(pScene->getPruner()->name());
    setInterpolator(pScene->getInterpolator()->name());
    setNavigation(pScene->getNavigation());
    setNavigator(pScene->getNavigation()->getNavigator().get());
    auto keys = pScene->getObstacleKeys();
    for(const auto& key: keys)
      setObstacle(key);
    keys = pScene->getPathKeys();
    for(const auto& key: keys)
      addPathCollection(key);

    const auto N = mpModels->pWorldVisModel->numberOfAlgorithmOutputs();
    for (size_t i(0); i<N; ++i)
    {
      auto* pVis = mpModels->pWorldVisModel->getAlgorithmOutput(i);
      addAlgorithmOutput(pVis->getName());
    }

    auto* pVisScene = mpModels->pVisModel->getVisScene();
    keys = pVisScene->getAbstractObjectKeys();
    for(const auto& key: keys)
      addAbstractObject(key);

    mpSceneWidget->expandAll();
  }

  /**
  */
  void PropertyController::addAbstractObject(const std::string& name)
  {
    std::vector<std::string> v;
    v.push_back(PropertyTypeNames[enAbstractObject]);
    v.push_back(name);
    mpSceneWidget->setItem(v);
  }
  
  /**
  */
  void PropertyController::deleteAbstractObject(const std::string& name)
  {
    std::vector<std::string> v;
    v.push_back(PropertyTypeNames[enAbstractObject]);
    v.push_back(name);
    mpSceneWidget->removeItem(v);
  }

  /**
  */
  void PropertyController::addAlgorithmOutput(const std::string& name)
  {
    auto* item = mpSceneWidget->topLevelItem(enAlgorithmOutput);
    const auto N = item->childCount();
    bool found = false;
    for (size_t i(0); i<N; ++i)
    {
      if (item->child(i)->text(0).toStdString() == name)
        found = true;
    }
    if (!found)
    {
      std::vector<std::string> v;
      v.push_back(PropertyTypeNames[enAlgorithmOutput]);
      v.push_back(name);
      mpSceneWidget->setItem(v);
    }
  }

  /**
  */
  void PropertyController::deleteAlgorithmOutput(const std::string& name)
  {
    std::vector<std::string> v;
    v.push_back(PropertyTypeNames[enAlgorithmOutput]);
    v.push_back(name);
    mpSceneWidget->removeItem(v);
  }

  /**
  */
  void PropertyController::addPathCollection(const std::string& name)
  {
    std::vector<std::string> v;
    v.push_back(PropertyTypeNames[enPathCollection]);
    v.push_back(name);
    mpSceneWidget->setItem(v);
  }

  /**
  */
  void PropertyController::deletePathCollection(const std::string& name)
  {
    std::vector<std::string> v;
    v.push_back(PropertyTypeNames[enPathCollection]);
    v.push_back(name);
    mpSceneWidget->removeItem(v);
  }

  /**
  */
  void PropertyController::setObstacle(const std::string& name)
  {
    std::vector<std::string> v;
    v.push_back(PropertyTypeNames[enObstacle]);
    v.push_back(name);
    mpSceneWidget->setItem(v);
    Vec3d color = mpModels->pAppModel->getScene()->getObstacle(name)->getActive() ? Vec3d(Colors::Green) : Vec3d(Colors::Red);
    mpSceneWidget->setColor(v, color);
  }

  /** \brief apparently delets osbtacle items in the scenewidget
  */
  void PropertyController::deleteObstacle(const std::string& name)
  {
    std::vector<std::string> v;
    v.push_back(PropertyTypeNames[enObstacle]);
    v.push_back(name);
    mpSceneWidget->removeItem(v);
  }

  /**
  */
  void PropertyController::deleteAbstractObject()
  {
    const auto items = mpSceneWidget->selectedItems();
    if (items.empty() || items.size() > 1)
      return;
    const auto* item = items.back();
    auto* parent = item->parent();
    if (nullptr == parent)
      return;

    EnPropertyType type = EnPropertyType(mpSceneWidget->indexOfTopLevelItem(parent));
    {
      const auto errmsg = "only segmentations and obstacles can be saved!"; 
      if (type != enAbstractObject)
        throw MUK_EXCEPTION_SIMPLE(errmsg);
    }
    const std::string name = item->text(0).toLocal8Bit().constData();
    mpControls->mpVisControl->deleteAbstractObject(name);
  }

  /**
  */
  void PropertyController::setPlanner(const std::string& name)
  {      
    mpSceneWidget->setTopLevelItem(enPlanner, name);
  }

  /**
  */
  void PropertyController::setPruner(const std::string& name)
  {
    mpSceneWidget->setTopLevelItem(enPruner, name);
  }

  /**
  */
  void PropertyController::setInterpolator(const std::string& name)
  {
    mpSceneWidget->setTopLevelItem(enInterpolator, name);
  }

  /**
  */
  void PropertyController::setNavigation(const INavigationContainer* const pNavigator)
  {
    // Race Condition, if setName accesses any non-static information
    mpSceneWidget->setTopLevelItem(enNavigator, pNavigator->name());
  }

  /**
  */
  void PropertyController::setNavigator(ProtectedNavigator* pObj)
  {
    // as per convention will always be child one
    mpSceneWidget->setTopLevelItem(enNavigator, pObj->name());
  }

  /**
  */
  void PropertyController::reloadProperty()
  {
    auto items = mpSceneWidget->selectedItems();
    // save old configuration
    std::pair<int, std::list<std::string>> mOldItem;
    if ( ! items.empty())
    {
      QTreeWidgetItem* pItem = items[0];
      QTreeWidgetItem* pParent = pItem->parent();
      while(pParent != nullptr)
      {
        mOldItem.second.push_front(pItem->text(0).toLocal8Bit().constData());
        pParent = pParent->parent();
        pItem   = pItem->parent();
      }
      mOldItem.first = mpSceneWidget->indexOfTopLevelItem(pItem);
    }
    initSceneWidget();
    // try to load configuration
    if ( ! items.empty())
    {
      QTreeWidgetItem* pItem = mpSceneWidget->topLevelItem(mOldItem.first);
      for (const auto& string : mOldItem.second)
      {
        pItem = gris::muk::findChild(pItem, string);
      }
      if (pItem != nullptr)
        mpSceneWidget->setItemSelected(pItem, true);
    }
    else
    {
      showProperty(nullptr, 0);
    }
  }
  
  /** \brief clears PropertyWindow, retrieves Properties of pItem and displays those

    implements logic to retrieve multiple properties (plural ! ) from qwidgetitem
    
    e.g. path has path properties and visualization properties (todo: really ?)
  */
  void PropertyController::showProperty(QTreeWidgetItem* pItem, int column)
  {
    mpPropertyWidget->clear();
    if (nullptr == pItem)
      return;
    auto* pParent = pItem->parent();
    auto& type = mCachedType;
    if (pParent == nullptr)
    {
      type = EnPropertyType(mpSceneWidget->indexOfTopLevelItem(pItem));
      if (!(type==enPlanner || type==enPruner || type==enInterpolator || type==enNavigator))
        return;
    }
    else
    {
      type = EnPropertyType(mpSceneWidget->indexOfTopLevelItem(pParent));
    }
    // generate hierarchical list of properties
    mCachedProperties = mpModels->pPropModel->listProperties(type, pItem->text(0).toLocal8Bit().constData());
    auto propTuples = mpModels->pPropModel->listPropertyTuples(mCachedProperties);
    for (const auto& tuple: propTuples)
    {
      mpPropertyWidget->addTopLevelProperty(tuple.mName);
      appendToWidget(tuple);
    }
    mpPropertyWidget->expandAll();
  }

  /**
  */
  void PropertyController::showProperty(EnPropertyType type, gstd::DynamicProperty* pObj)
  {
	  mpPropertyWidget->clear();
	  if (nullptr == pObj)
		  return;

	  if (!(type==enAlgorithm))
		  return;

	  mCachedProperties = mpModels->pPropModel->listProperties(type, pObj);
	  auto propTuples = mpModels->pPropModel->listPropertyTuples(mCachedProperties);
	  for (const auto& tuple : propTuples)
	  {
		  mpPropertyWidget->addTopLevelProperty(tuple.mName);
		  appendToWidget(tuple);
	  }
	  mpPropertyWidget->expandAll();
  }

  /**
  */
  void PropertyController::appendToWidget(const AuxPropertyTuples& auxtuples)
  {
    for (const auto& tuple : auxtuples.mProperties)
    {
      using H = EnPropertyHint;
      const auto& name = std::get<0>(tuple);
      const auto& value = std::get<1>(tuple);
      const auto& hint = std::get<2>(tuple);
      switch(hint)
      {
        case H::enScalar:
        {
          mpPropertyWidget->addString(name, value);
          break;
        }
        case H::enString:
        {
          mpPropertyWidget->addString(name, value);
          break;
        }
        case H::enBool:
        {
          mpPropertyWidget->addBool(name, boost::lexical_cast<bool>(value));
          break;
        }
        case H::enColor:
        {
          Vec3d color;
          std::istringstream iss(value);
          iss >> color;
          mpPropertyWidget->addColor(name, color);
          break;
        }
        case H::enStringSelection:
        {
          mpPropertyWidget->addStringSelection(name, value, mStringSelections[name]);
          break;
        }
        default:
          LOG_LINE << "property hint not supported: " << name << " " << value;
      }
    }
    for (const auto& subProperty : auxtuples.mSubProperties)
    {
      mpPropertyWidget->addProperty(subProperty.mName);
      appendToWidget(subProperty);
      mpPropertyWidget->parentProperty();
    }
  }

  /**
  */
  void PropertyController::initialize()
  {
    mpSceneWidget = mpMainWindow->mpSceneWidget;
    mpPropertyWidget = mpMainWindow->mpPropertyWidget;
    initSceneWidget();
  }
  
  /**
  */
  void PropertyController::updateProperty()
  {
    auto vecParentProps = mpPropertyWidget->getParentProperties();
    if (vecParentProps.empty())
      return;
    const auto& topLevel = vecParentProps.front();
    std::vector<AuxProperty>::const_iterator pIter = std::find_if(mCachedProperties.begin(), mCachedProperties.end(), [&] (const AuxProperty& auxprop) { return auxprop.mName == topLevel; });
    if (pIter == mCachedProperties.end())
    {
      std::string info = "Expected: " + topLevel;
      throw MUK_EXCEPTION("Cache and PropertyWidget's return structure do not match", info.c_str());
    }
    for (size_t i(1); i<vecParentProps.size(); ++i)
    {
      const auto& subs = pIter->mSubProperties;
      pIter = std::find_if(subs.begin(), subs.end(), [&] (const AuxProperty& auxprop) { return auxprop.mName == vecParentProps[i]; });
      if (pIter == subs.end())
      {
        std::string info = "Expected: " + topLevel;
        throw MUK_EXCEPTION("Cache and PropertyWidget's return structure do not match", info.c_str());
      }
    }
    mpModels->pPropModel->updateProperty(mCachedType, pIter->mProp, mpPropertyWidget->getName(), mpPropertyWidget->getValue());
    if (mpPropertyWidget->getName() == "Active")
    {
      emit activChanged();
    }
    initSceneWidget();
    mpModels->pVisModel->render();
  }

  /**
  */
  void PropertyController::showBounds()
  {
    mpControls->mpVisControl->showBounds();
  }

  /**
  */
  void PropertyController::showTrajectory()
  {
    const auto& key = mpModels->pPlanningModel->getActivePathCollection();
    const auto& idx = mpModels->pPlanningModel->getActivePathIdx();
    mpControls->mpVisControl->addTrajectory(key, idx);
  }

  /**
  */
  void PropertyController::extractPlanningGraph()
  {
    std::string key = "Roadmap_" + mpModels->pPlanningModel->getActivePathCollection();
    std::unique_ptr<MukPathGraph> pGraph = mpModels->pPlanningModel->extractPlanningGraph();
    std::shared_ptr<VisMukPathGraph> pObj;
    auto keys = mpModels->pVisModel->getAbstractObjectKeys();
    if (std::any_of(keys.begin(), keys.end(), [&] (const auto& str) {return str == key; }))
    {
      pObj = std::dynamic_pointer_cast<VisMukPathGraph>(mpModels->pVisModel->getVisScene()->getObject(key));
    }
    else
    {
      pObj = std::make_shared<VisMukPathGraph>(key);
      mpControls->mpVisControl->addAbstractObject(pObj);
    }
    pObj->setData(*pGraph);
    pObj->setVisibility(true);
    mpModels->pVisModel->render();
  }

  /** \brief Saves a segmentation object or obstacle as .vtk data

    checks, if a valid object is selected
    then opens a save file dialog.
    if everything is ok, the object is saved
  */
  void PropertyController::saveObject() const
  {
    const auto items = mpSceneWidget->selectedItems();
    if (items.empty() || items.size() > 1)
      throw MUK_EXCEPTION_SIMPLE("no item or multiple items selected!");
    const auto* item   = items.back();
    auto* parent = item->parent();
    EnPropertyType type;
    const auto errmsg = "only segmentations and obstacles can be saved!";
    if (parent == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE(errmsg);
    }
    else
    {
      type = EnPropertyType(mpSceneWidget->indexOfTopLevelItem(parent));
      if (type != enObstacle && type != enAlgorithmOutput)
        throw MUK_EXCEPTION_SIMPLE(errmsg);
    }
    const std::string name = item->text(0).toLocal8Bit().constData();
    vtkPolyData* pData = nullptr;
    if (type == enAlgorithmOutput)
      pData = mpModels->pWorldVisModel->getAlgorithmOutput(name)->getData();
    else
      pData = mpModels->pVisModel->getVisScene()->getObstacle(name)->getData();
    
    QString qFilename = QFileDialog::getSaveFileName(nullptr, tr("Save as"),
      (boost::format("./%s.vtk") % name).str().c_str(), tr("vtk poyldata (*.vtk)"));
    if (qFilename.isEmpty())
      return;

    auto writer = make_vtk<vtkPolyDataWriter>();
    writer->SetInputData(pData);
    writer->SetFileName(qFilename.toLocal8Bit().constData());
    writer->Update();
  }

  /** \brief deletes an obstacle from the scene and updates the scene widget
  */
  void PropertyController::deleteObstacle()
  {
    const auto items = mpSceneWidget->selectedItems();
    if (items.empty() || items.size() > 1)
      throw MUK_EXCEPTION_SIMPLE("no item or multiple items selected!");
    const auto* item   = items.back();
    auto* parent = item->parent();
    EnPropertyType type;
    const auto errmsg = "only segmentations and obstacles can be saved!";
    if (parent == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE(errmsg);
    }
    else
    {
      type = EnPropertyType(mpSceneWidget->indexOfTopLevelItem(parent));
      if (type != enObstacle)
        throw MUK_EXCEPTION_SIMPLE(errmsg);
    }
    const std::string name = item->text(0).toLocal8Bit().constData();
    mpControls->mpPlanningController->deleteObstacle(name);
    deleteObstacle(name);
  }

  /**
  */
  void PropertyController::updateNavigator()
  {
    auto* pScene = mpModels->pAppModel->getScene().get();
    setNavigation(pScene->getNavigation());
    setNavigator(pScene->getNavigation()->getNavigator().get());
  }

  /**
  */
  void PropertyController::evaluateDroppedText(const std::string& text)
  {
    if (text.substr(0, 4) != "file")
    {
      return;
    }

    namespace fs = boost::filesystem;
    const auto path = fs::path(text.substr(8));
    if ( ! fs::is_regular_file(path))
      return;

    auto ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext != ".mukscene")
    {
      return;
    }
    mpControls->mpToolbar->loadScene(path.string());
  }
}
}
