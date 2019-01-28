#pragma once
#include "BaseModel.h"

#include "gstd/DynamicProperty.h"

#include <vtkSmartPointer.h>
class vtkImageData;
class vtkPolyData;

namespace gris
{
  namespace muk
  {
    class SliceWidget;
    class VisAbstractObject;
    class VtkWindow;
  
    /**
    */
    class MUK_APP_API WorldVisualizationModel : public BaseModel
    {
      public:
        WorldVisualizationModel();
        virtual ~WorldVisualizationModel();
    
      public:
        static  const char* s_name()     { return "AlgorithmVisualizationModel"; }
        virtual const char* name() const { return s_name(); }

      public:
        void initialize();
        void render();

        // image and widget stuff
      public:
        void setAxialSliceWidget(SliceWidget* pWidget);
        void setSagittalSliceWidget(SliceWidget* pWidget);
        void setCoronalSliceWidget(SliceWidget* pWidget);
        void set3DWindow(VtkWindow* pWidget);

        void setCtImage(vtkImageData* pImage);
        void setSegmentationImage(vtkImageData* pImage);

        vtkImageData* getCtImage() const;

        // 3D visualization of algorithm outputs
      public:
        void   setSegmentationObject(const std::string& name, vtkSmartPointer<vtkPolyData>& pObj);
        void   deleteSegmentationObject(const std::string& name);
        size_t numberOfAlgorithmOutputs() const;
        VisAbstractObject* getAlgorithmOutput(size_t i) const;
        VisAbstractObject* getAlgorithmOutput(const std::string& name) const;

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}