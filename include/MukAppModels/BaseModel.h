#pragma once
#include "AppModels.h"
#include "muk_appmodels_api.h"

#include <gstd/DynamicProperty.h>

namespace gris
{
  namespace muk
  {
    /** \brief base class for all application models

      unit interface to enforce proper initialization of const pointer to other models
      
      each model will only be allowed to change its own members
      each model shall have access to const functions of other models.
    */
    class MUK_APP_API BaseModel : public gstd::DynamicProperty
    {
      public:
        void setModels(const AppModels* pModels) { mpModels = pModels; }

    public:
        virtual const char* name() const = 0;

      protected:
        const AppModels* mpModels = nullptr;
    };

  }
}