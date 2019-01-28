#pragma once

#include "muk_visualization_api.h"

#include "VisualProp.h"

#include "MukCommon/MukTransform.h"

#include <vtkAxesActor.h>

namespace gris
{
  namespace muk
  {
      class VisCoordinateSystem : public VisualProp
      {
      public:
        static const int FONT_SIZE;
        static const double MAX_SCALE;
        static const double MIN_SCALE;

      public:
        VisCoordinateSystem();
        // copying VisCoordinateSystem Objects leads to errors due to the removing of the Actors in the Destructor
        // Since Label and Actor are SmartPointers, they are properly "disposed" of.
        VisCoordinateSystem(const VisCoordinateSystem&) = delete;
        VisCoordinateSystem(const VisCoordinateSystem&&) = delete;
        ~VisCoordinateSystem();

      public:
        virtual void update();

        vtkAxesActor* getAxesActor();
        vtkCaptionActor2D* getLabel();

        void setName(const std::string& name);

        virtual void setRenderer(vtkRenderer* pRenderer);

        virtual void setTransform(const vtkSmartPointer<vtkTransform> transform);

        void fade(const double p);
        void setVisibility(const bool state);
        bool isVisible() const;
      private:
        vtkSmartPointer<vtkCaptionActor2D> mpLabel;
      };

  }
}