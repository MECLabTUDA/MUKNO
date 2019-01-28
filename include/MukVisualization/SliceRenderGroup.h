#pragma once
#include "muk_visualization_api.h"

#include "MukCommon/MukVector.h"

#include <vtkSmartPointer.h>
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkActor2D;
class vtkImageData;
class vtkImageMapper;

#include <memory>

namespace gris
{
  namespace muk
  {
    /**
    */
    enum MUK_VIS_API EnMedicalOrientation
    {
      enAxial,
      enSagittal,
      enCoronal
    };

    /**
    */
    struct MUK_VIS_API SegmentationLabel
    {
      SegmentationLabel() {}
      SegmentationLabel(const std::string& l, int v, const Vec3d& c)
        : label(l)
        , grayValue(v)
        , color(c)
      {
      }

      std::string label;
      int         grayValue;
      Vec3d       color;
    };

    /**
    */
    struct MUK_VIS_API SliceInfo
    {
      int    orientation;
      int    sliceNumber;
      int    maxSlices;
      double sliceCoord;
      double normal[3];
      double origin[3];
    };

    class Cursor3DWidget;
    
    /**
    */
    class MUK_VIS_API SliceRenderGroup
    {
      public:
        SliceRenderGroup();
        virtual ~SliceRenderGroup();

      public:
        void setRenderWindow(vtkRenderWindow* window);
        void setImage(vtkImageData* pImage);
        void setSegmentationImage(vtkImageData* pImage);
        void setOrientation(EnMedicalOrientation type);
        void setLookupTable(const std::vector<SegmentationLabel>& labels);
        void setCurrentSlice(double position[3]);

        vtkImageData*   getImage() const;
        Cursor3DWidget* getCursor() const;
        SliceInfo       getCurrentSlice() const;

        void showSegmentation(bool b);
        void zoomToFit();

      public:
        void    setColorLevel(double val);
        double  getColorLevel()          const;
        void    setColorWindow(double val);
        double  getColorWindow()          const;

      public:
        vtkRenderer*      getRenderer();
        vtkRenderWindow*  getRenderWindow();
        vtkRenderWindowInteractor* getInteractor();

      private:
        struct Impl;
        std::unique_ptr<Impl> mp;
    };
  }
}
