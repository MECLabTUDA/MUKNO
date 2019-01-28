#pragma once

#include "VisualObject.h"

#include "vtkTransform.h"
#include "vtkSmartPointer.h"

namespace gris
{
  namespace muk
  {
    /**
    */
    class MUK_VIS_API VisAbstractObject : public VisualObject
    {
      public:
        explicit VisAbstractObject(const std::string& name);
        virtual ~VisAbstractObject();

      public:
        void                            setTransform(const vtkSmartPointer<vtkTransform> vector);
        vtkSmartPointer<vtkTransform>   getTransform() const;
    
      public:
        virtual void update() {}

      public:
        void                setName(const std::string& name);
        const std::string&  getName()                           { return mName; }

      private:
        void assignTransformToActor();

      private:
        vtkSmartPointer<vtkTransform> mpTransform;
        std::string                   mName;
    };

  }
}
