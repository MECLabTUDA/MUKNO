#pragma once
#include "MukCommon/MukException.h"

#include "gstd//DynamicProperty.h"

#include <string>


namespace gris
{
namespace muk
{
  /** \brief Supported Properties that can be visualized and changed in the PropertyWindow
  */
  enum EnPropertyType
  {
    enAppModels,
    enObstacle,
    enPathCollection,
    enPlanner,
    enPruner,
    enInterpolator,
    enNavigator,
    enAlgorithmOutput,
    enAbstractObject,
    enSize_Property_Type,
    enAlgorithm
  };

  /** \brief Names of types of properties that will appear in the GUI's SceneWidget
  */
  const char* const PropertyTypeNames[enSize_Property_Type] =
  {
    "Models",
    "Obstacles",
    "PathCollections",
    "Planner",
    "Pruner",
    "Interpolator",
    "Navigator",
    "AlgorithmOutput",
    "AbstractObjects",
  };

  EnPropertyType fromName(const std::string& str);
  
  /** \brief Defines the way, the PropertyWidget will display a property

    QTextEdit: enScalar and enString
    Checkbox:  enBool
    ColorWindow: enColor
  */
  enum class EnPropertyHint
  {
    enScalar,
    enString,
    enBool,
    enColor,
    enStringSelection
  };

  /**
  */
  EnPropertyHint extractHintFromPropertyName(const std::string& name);

  /** \brief wrapper for the incomplete implementation of gstd::DynamicProperty

    saves pointers to properties
  */
  struct AuxProperty
  {
    AuxProperty() {}
    AuxProperty(const std::string& str, gstd::DynamicProperty* prop_)
      : mName(str), mProp(prop_)
    {}

    std::string mName;
    gstd::DynamicProperty* mProp;
    std::vector<AuxProperty> mSubProperties;
  };

  /** \brief wrapper for the incomplete implementation of gstd::DynamicProperty

    build up a graph with leaves of the form (PropName, PropVal, PropHint) or (<string, string, hint>)
  */
  struct AuxPropertyTuples
  {
    AuxPropertyTuples() {}
    AuxPropertyTuples(const std::string& str)
      : mName(str)
    {}

    void addProperty(gstd::DynamicProperty* prop);
    void addProperty(const std::string& name, gstd::DynamicProperty* prop);

    std::string mName;
    using PropTuple = std::tuple<std::string, std::string, EnPropertyHint>;
    std::vector<PropTuple> mProperties;
    std::vector<AuxPropertyTuples> mSubProperties;
  };
}
}