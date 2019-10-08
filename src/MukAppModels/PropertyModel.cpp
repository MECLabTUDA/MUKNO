#include "private/muk.pch"
#include "private/PropertyDetails.h"
#include "PropertyModel.h"

#include "AlgorithmModel.h"
#include "ApplicationModel.h"
#include "LocalEnvironment.h"
#include "PlanningModel.h"
#include "VisualizationModel.h"
#include "WorldVisualizationModel.h"

#include "MukCommon/MukException.h"
#include "MukCommon/MukScene.h"
#include "MukCommon/INavigator.h"
#include "MukCommon/PathCollection.h"
#include "MukCommon/MukStringToolkit.h"

#include "MukVisualization/VisAbstractObject.h"
#include "MukVisualization/VisObstacle.h"
#include "MukVisualization/VisPathCollection.h"
#include "MukVisualization/VisScene.h"

#include <qtreewidgetitem>
#include <qcheckbox.h>
#include <qcombobox.h>
#include <qpushbutton.h>

#include <qflags.h>
#include <QColorDialog>
#include <QCheckBox>
#include <qcomboBox.h>
#include <qobject.h>

#include <boost/format.hpp>

#include <utility>

namespace
{
  using namespace gris::muk;

  void traverseProperty(const AuxProperty& auxProp, AuxPropertyTuples& propTuple)
  {
    propTuple.mName = auxProp.mName;
    propTuple.addProperty(auxProp.mProp);
    const auto N = auxProp.mSubProperties.size();
    propTuple.mSubProperties.resize(N);
    for (size_t i(0); i<N; ++i)
    {
      traverseProperty(auxProp.mSubProperties[i], propTuple.mSubProperties[i]);
    }
  }
}

namespace gris
{
namespace muk
{
  /**
  */
  PropertyModel::PropertyModel()
    : BaseModel()
  {
  }

  /**
  */
  PropertyModel::~PropertyModel()
  {
  }

  /** \brief generates a hierarchical list of DynamicProperty object pointers.

    this is done to omly compute the hierarchy of DynamicProperty objects. 
    Their properties are not (!) retrieved here.

    e.g. input enPlanner, "ManualPlanner" generates only one AuxProperty with
      name = "Planer";
      mProp = mpScene->getPlanner();
      mSubProperties = 'empty'

    e.g. input enObstacle, "FacialNerve" generates two AuxProperty with
      mName = Obstacle;
      mProp = pScene->getObstacle(name).get();

      mName = VisObstacle;
      mProp = pVisScene->getObstacle(name).get();
  */
  std::vector<AuxProperty> PropertyModel::listProperties(EnPropertyType type, const std::string& name)
  {
    auto* pScene = mpModels->pAppModel->getScene().get();
    auto* pVisScene = mpModels->pVisModel->getVisScene();
    std::vector<AuxProperty> result;
    switch (type)
    {
      case enAppModels:
      {
        if ( ! mpModels->hasModel(name))
          throw MUK_EXCEPTION("no model with this name available", name.c_str());
        auto* pObj = mpModels->getModel(name);
        result.push_back(AuxProperty(pObj->name(), pObj));
        break;
      }
      case enObstacle:
      {
        AuxProperty prop;
        prop.mName = "Obstacle";
        prop.mProp = pScene->getObstacle(name).get();
        result.push_back(prop);
        prop.mName = "VisObstacle";
        prop.mProp = pVisScene->getObstacle(name).get();
        result.push_back(prop);
        break;
      }
      case enPathCollection:
      {
        AuxProperty prop;
        prop.mName = "PathCollection";
        prop.mProp = &pScene->getPathCollection(name);
        auto pVisColl = pVisScene->getPathCollection(name);
        prop.mSubProperties.push_back(AuxProperty());
        AuxProperty& propdefProp = prop.mSubProperties.back();
        {
          propdefProp.mName = "ProblemDefinition";
          propdefProp.mProp = pScene->getPathCollection(name).getProblemDefinition().get();
          for (size_t i(0); i<pVisColl->sizeStart(); ++i)
          {
            AuxProperty subprop;
            subprop.mName = pVisColl->getStartRegion(i).getName();
            subprop.mProp = &pVisColl->getStartRegion(i);
            propdefProp.mSubProperties.push_back(subprop);
          }
          for (size_t i(0); i<pVisColl->sizeGoal(); ++i)
          {
            AuxProperty subprop;
            subprop.mName = pVisColl->getGoalRegion(i).getName();
            subprop.mProp = &pVisColl->getGoalRegion(i);
            propdefProp.mSubProperties.push_back(subprop);
          }
          /*for (size_t i(0); i<pVisColl->sizeWaypoints(); ++i)
          {
            AuxProperty subprop;
            subprop.mName = "WP" + std::to_string(i);
            subprop.mProp = &pVisColl->getWaypoint(i);
            propdefProp.mSubProperties.push_back(subprop);
          }*/
        }
        auto N_Paths = pVisColl->numberOfPaths();
        for (size_t i(0); i<N_Paths; ++i)
        {
          AuxProperty subprop;
          subprop.mName = "Path" + std::to_string(i);
          subprop.mProp = pVisColl->getMukPath(i).get();
          prop.mSubProperties.push_back(subprop);
        }
        result.push_back(prop);
        break;
      }
      case enAbstractObject:
      {
        result.push_back(AuxProperty("AbstractObject", pVisScene->getObject(name).get()));
        break;
      }
      case enPlanner:
        result.push_back(AuxProperty("Planner", pScene->getPlanner()));
        break;
      case enPruner:
        result.push_back(AuxProperty("Pruner", pScene->getPruner()));
        break;
      case enInterpolator:
        result.push_back(AuxProperty("Interpolator", pScene->getInterpolator()));
        break;
      case enNavigator:
      {
        NavigatorProperty* pProperties = pScene->getNavigation()->getProperties();
        //  if (std::string(mpScene->getNavigation()->name()) == name)
        //    // this is the "main" entry --> currently inactive
        //    return;
        // only actually add anything, if it is a valid pointer
        if (pProperties != nullptr)
        {
          result.push_back(AuxProperty("Navigator", pProperties));
        }
        break;
      }
      case enAlgorithmOutput:
      {
        auto* pObj = mpModels->pWorldVisModel->getAlgorithmOutput(name);
        if (pObj)
          result.push_back(AuxProperty("AbstractObject", pObj));
        break;
      }
    }
    return result;
  }

  /**
  */
  std::vector<AuxProperty> PropertyModel::listProperties(EnPropertyType type, gstd::DynamicProperty* pObj)
  {
	  std::vector<AuxProperty> result;
	  switch (type)
	  {	  
		  case enAlgorithm:
			  result.push_back(AuxProperty("Algorithm", pObj));
			  break;
	  }
	  return result; 
  }

  /** \brief generates from a hierarchical list a hierarchical list of the actual properties with name, value and hint.

  This is passed to the PropertyWidget.

  e.g. properties of IPathPlanner
  name = "Planer";  // from input 'listProperties'
    // these are created by this function
    tuple ("Step-Size", "5", string)           
    tuple ("Calc-Time", "0.25", string)
    tuple ("useOptimization", "true", boolean) // does not exist, just example
  mSubProperties = 'empty'

  e.g. properties of Obstacle
  name = "Obstacle";  // from input 'listProperties'
    // these are created by this function
    tuple ("Active", "true", boolean)           
    tuple ("File", "D:/...", string)
    tuple ("Name", "NervusFacialis", string)
  name = "VisObstacle";  // from input 'listProperties'
    // these are created by this function
    tuple ("Color", "0.1 0.1 0.5", color)           
    tuple ("Opacity", "0.5", string)
    tuple ("Visibility", "true", string)
  */
  std::vector<AuxPropertyTuples> PropertyModel::listPropertyTuples(std::vector<AuxProperty>& listProperties)
  {
    std::vector<AuxPropertyTuples> ret;
    const auto N = listProperties.size();
    ret.resize(N);
    for (size_t i(0); i<N; ++i)
    {
      traverseProperty(listProperties[i], ret[i]);
    }
    return ret;
  }

  /** \brief sets the property and calls further functions

    properties are not yet implemented correctly, some adjustments have still to be made.
  */
  void PropertyModel::updateProperty(EnPropertyType type, gstd::DynamicProperty* prop, const std::string& name, const std::string& value)
  {
    prop->setProperty(name, value);
    switch (type)
    {
      case enObstacle:
      {
        if (name == "Active")
        {
          std::string obstacleName;
          prop->getProperty("Name", obstacleName);
          auto* detector = mpModels->pAppModel->getScene()->getCollisionDetector().get();
          bool b = detector->isActive(obstacleName);
          detector->setActive(obstacleName, ! b);
          detector->rebuild();
          auto key = mpModels->pPlanningModel->getActivePathCollection();
          if ( ! key.empty())
          {
            if (!b)
            {
              mpModels->pAppModel->getScene()->getPathCollection(key).removeObstacle(obstacleName);
            }
            else
            {
              mpModels->pAppModel->getScene()->getPathCollection(key).addObstacle(obstacleName);
            }
          }
        }
        break;
      }
    }
  }
  
}
}