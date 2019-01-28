#pragma once

#include "VisAbstractObject.h"

#include "vtkTransform.h"
#include "vtkSmartPointer.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisScaleableObject : public VisAbstractObject
    {
      public:
        explicit VisScaleableObject(const std::string& name);
        virtual ~VisScaleableObject();

      public:
        virtual void update();

      public: 
        virtual void setScale(const float scale);
        float getScale() const;

      protected:
        void declareProperties();
    };

  }
}
