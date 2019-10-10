#pragma once
#include "BaseModel.h"

#include "MukQt/PropertyWidget.h"
#include "MukQt/SceneWidget.h"

#include "gstd/DynamicProperty.h"

namespace gris
{
  namespace muk
  {
    struct AuxProperty;
    struct AuxPropertyTuples;
    enum EnPropertyType;
    enum class EnPropertyHint;

    /**
    */
    class MUK_APP_API PropertyModel : public BaseModel
    {
      public:
        explicit PropertyModel();
        ~PropertyModel();

      public:
        virtual const char* name() const { return "PropertyModel"; }

      public:
        std::vector<AuxProperty>        listProperties(EnPropertyType type, const std::string& name);
        std::vector<AuxProperty>        listProperties(EnPropertyType type, gstd::DynamicProperty* pObj);
        std::vector<AuxPropertyTuples>  listPropertyTuples(std::vector<AuxProperty>& listProperties);
        void updateProperty(EnPropertyType type, gstd::DynamicProperty* prop, const std::string& name, const std::string& value);
        
      private:
        std::vector<EnPropertyHint> mCurrentHints;
    };

  }
}
