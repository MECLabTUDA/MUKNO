#include "private/muk.pch"
#include "private/PropertyDetails.h"

namespace gris
{
namespace muk
{
  /**
  */
  EnPropertyType fromName(const std::string& str)
  {
    for (int i(0); i<enSize_Property_Type; ++i)
    {
      if (str == PropertyTypeNames[i])
        return EnPropertyType(i);
    }
    return EnPropertyType::enSize_Property_Type;
  }

  /**
  */
  EnPropertyHint extractHintFromPropertyName(const std::string& name)
  {
    EnPropertyHint ret;
    static std::vector<std::string> booleanProperties = {"OrientationMarkerVisibility","Visibility", "Active", "F: ", "Fixed interaction", "Interactor", "Orthoradial Slices", "SwitchStartAndGoal"};
    static std::vector<std::string> colorProperties   = {"Color", "Color1", "Color2", "DefaultSphereColor", "WarningColor"};
    static std::vector<std::string> stringCollectionProperties = { "Topology", "DefaultPathTopology", "InterpolationType", "DisplayType"};

    if (std::any_of(booleanProperties.begin(), booleanProperties.end(), [&] (const auto& str) { return name ==str; }))
    {
      ret = EnPropertyHint::enBool;
    }
    else if (std::any_of(colorProperties.begin(), colorProperties.end(), [&] (const auto& str) { return  name ==str; }))
    {
      ret = EnPropertyHint::enColor;
    }
    else if (std::any_of(stringCollectionProperties.begin(), stringCollectionProperties.end(), [&] (const auto& str) { return name ==str; }))
    {
      ret = EnPropertyHint::enStringSelection;
    }
    else if (name.empty())
    {
      throw MUK_EXCEPTION_SIMPLE("Empty strings are not supported!"); // should probably not be supported, so throw
    }
    else
    {
      ret = EnPropertyHint::enString;
    }
    return ret;
  }

  /**
  */
  void AuxPropertyTuples::addProperty(gstd::DynamicProperty* prop)
  { 
    std::vector<std::string> propNames;
    prop->getPropertyNames(propNames);
    mProperties.resize(propNames.size());
    for (size_t i(0); i<propNames.size(); ++i)
    {
      auto& key = std::get<0>(mProperties[i]);
      auto& val = std::get<1>(mProperties[i]);
      auto& hint = std::get<2>(mProperties[i]);
      key = propNames[i];
      prop->getProperty(key, val);
      hint = extractHintFromPropertyName(key);
    }    
  }

  /**
  */
  void AuxPropertyTuples::addProperty(const std::string& name, gstd::DynamicProperty* prop)
  {
    mName = name;
    addProperty(prop);
  }
}
}