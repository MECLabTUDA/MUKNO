#include "private/muk.pch"
#include "SliceRenderGroup.h"

#include "Cursor3DRepresentation.h"
#include "Cursor3DWidget.h"
#include "SliceInteractorStyle.h"

#include "MukCommon/muk_common.h"

#include <vtkActor2D.h>
#include <vtkCamera.h>
#include <vtkImageData.h>
#include <vtkImageMapToColors.h>
#include <vtkImageProperty.h>
#include <vtkImageSliceMapper.h>
#include <vtkImageSlice.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLookUpTable.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

namespace gris
{
  namespace muk
  {
    /**
    */
    struct SliceRenderGroup::Impl 
    {
      Impl();

      void initialize();

      vtkSmartPointer<vtkImageData> mpImage;			// 3d image containing the greyscale Ct image 
      vtkSmartPointer<vtkImageData> mpLabelImage; // 3d image containing segmentations
      vtkSmartPointer<vtkImageData> mpOverlayImage; // 3d image with all pixels set to RGBA 0000

      vtkSmartPointer<vtkImageSliceMapper> mpImageMapper;
      vtkSmartPointer<vtkImageSliceMapper> mpLabelMapper;
      vtkSmartPointer<vtkImageSliceMapper> mpOverlayMapper;

      vtkSmartPointer<vtkImageSlice>  mpImageActor;		    // actor to display the grayscale ct image 
      vtkSmartPointer<vtkImageSlice>  mpLabelActor;	// actor to display the risk structures
      vtkSmartPointer<vtkImageSlice>  mpOverlayActor;      // actor to display other stuff like the axes of the cursor

      vtkSmartPointer<vtkLookupTable>  mpLookupTable;
      vtkSmartPointer<vtkImageMapToColors> mpTransparencyMap;

      vtkRenderWindow*             mpRenderWindow;
      vtkSmartPointer<vtkRenderer> mpRenderer;

      vtkSmartPointer<vtkRenderWindowInteractor> mpInteractor;
      vtkSmartPointer<SliceInteractorStyle> mpInteractorStyle;
      vtkSmartPointer<Cursor3DWidget>       mpCursorWidget;

      EnMedicalOrientation mOrientation;
    };

    /**
    */
    SliceRenderGroup::Impl::Impl()
      : mOrientation(enAxial)
    {
      mpRenderer = make_vtk<vtkRenderer>();
      mpRenderer->SetBackground(0,0,0);

      mpInteractor = make_vtk<vtkRenderWindowInteractor>();
      mpInteractorStyle = make_vtk<SliceInteractorStyle>();
      mpInteractor->SetInteractorStyle(mpInteractorStyle);
      mpCursorWidget = make_vtk<Cursor3DWidget>();
      mpCursorWidget->SetInteractor(mpInteractor);
      mpCursorWidget->GetRepresentation()->SetPlaceFactor( 1 );

      // basic members for the pipeline
      mpImage         = make_vtk<vtkImageData>();
      mpLabelImage    = make_vtk<vtkImageData>();
      mpOverlayImage  = make_vtk<vtkImageData>();
      mpImageMapper   = make_vtk<vtkImageSliceMapper>();
      mpLabelMapper   = make_vtk<vtkImageSliceMapper>();
      mpOverlayMapper = make_vtk<vtkImageSliceMapper>();
      mpImageActor    = make_vtk<vtkImageSlice>();
      mpLabelActor    = make_vtk<vtkImageSlice>();
      mpOverlayActor  = make_vtk<vtkImageSlice>();
      // functions that modify the image

      mpImageMapper->SetInputData(mpImage);
      mpImageMapper->SliceFacesCameraOn();
      mpImageMapper->SetSliceAtFocalPoint(true);
      mpImageActor->GetProperty()->SetInterpolationTypeToNearest();
      mpImageActor ->SetMapper(mpImageMapper);

      mpLookupTable = make_vtk<vtkLookupTable>();
      mpTransparencyMap = make_vtk<vtkImageMapToColors>();
      mpTransparencyMap->SetLookupTable(mpLookupTable);
      mpTransparencyMap->PassAlphaToOutputOn();
      mpTransparencyMap->SetInputData(mpLabelImage);

      mpLabelMapper->SetInputConnection(mpTransparencyMap->GetOutputPort());
      mpLabelMapper->SliceFacesCameraOn();
      mpLabelMapper->SetSliceAtFocalPoint(true);
      mpLabelActor ->SetMapper(mpLabelMapper);
      mpLabelActor ->GetProperty()->SetOpacity(0.5);
      mpLabelActor ->SetVisibility(false);

      mpOverlayMapper->SetInputData(mpOverlayImage);
      mpOverlayMapper->SliceFacesCameraOn();
      mpOverlayMapper->SetSliceAtFocalPoint(true);
      mpOverlayActor ->SetMapper(mpOverlayMapper);

      mpRenderer->AddViewProp(mpImageActor);
      mpRenderer->AddViewProp(mpLabelActor);
      mpRenderer->ResetCamera();
      mpRenderer->GetActiveCamera()->ParallelProjectionOn();
    }

    void SliceRenderGroup::Impl::initialize()
    {
      mpRenderWindow->AddRenderer(mpRenderer);
      mpInteractor->SetRenderWindow(mpRenderWindow);
    }

    // ==================================================================================

    /**
    */
    SliceRenderGroup::SliceRenderGroup()
      : mp(std::make_unique<Impl>())
    {
    }

    /**
    */
    SliceRenderGroup::~SliceRenderGroup()
    {
    }

    /**
    */
    void SliceRenderGroup::setRenderWindow(vtkRenderWindow* window)
    {
      mp->mpRenderWindow = window;
      mp->initialize();
    }

    /**
    */
    vtkImageData* SliceRenderGroup::getImage() const
    {
      return mp->mpImage;
    }

    /**
    */
    Cursor3DWidget* SliceRenderGroup::getCursor() const
    {
      return mp->mpCursorWidget;
    }

    /**
    */
    void SliceRenderGroup::setColorWindow(double val)
    {
      mp->mpImageActor->GetProperty()->SetColorWindow(val);
      mp->mpRenderWindow->Render();
    }

    /**
    */
    double SliceRenderGroup::getColorWindow() const
    {
      return mp->mpImageActor->GetProperty()->GetColorWindow();
    }

    /**
    */
    void SliceRenderGroup::setColorLevel(double val)
    {
      mp->mpImageActor->GetProperty()->SetColorLevel(val);
    }

    /**
    */
    double SliceRenderGroup::getColorLevel() const
    {
      return mp->mpImageActor->GetProperty()->GetColorLevel();
    }

    /**
    */
    void SliceRenderGroup::setOrientation(EnMedicalOrientation type)
    {
      mp->mOrientation = type;
      switch(type)
      {
        case enAxial:
          mp->mpInteractorStyle->setOrientationToZ();
          break;
        case enCoronal:
          mp->mpInteractorStyle->setOrientationToX();
          break;
        case enSagittal:
          mp->mpInteractorStyle->setOrientationToY();
          break;
      };
      mp->mpInteractor->Render();
    }

    /**
    */
    void SliceRenderGroup::setImage(vtkImageData* pImage)
    {
      mp->mpImage = pImage;
      mp->mpImageMapper->SetInputData(mp->mpImage);
      mp->mpImageMapper->Modified();
      mp->mpImageActor->GetProperty()->SetColorWindow(4000);
      mp->mpImageActor->GetProperty()->SetColorLevel(2000);
      mp->mpImageActor->Modified();
      showSegmentation(false);
      mp->mpRenderer->ResetCamera();
      mp->mpInteractorStyle->FindPokedRenderer(0, 0);
      setOrientation(mp->mOrientation);
      // update cursor
      if (pImage)
      {
        double d[6];
        pImage->GetBounds(d);
        mp->mpCursorWidget->GetRepresentation()->PlaceWidget( d );
        mp->mpCursorWidget->setImage(pImage);
        // internal update so the position is known
        mp->mpImageMapper->Update();
        double position[3];
        for (size_t i(0); i < 3; ++i)
        {
          auto info = getCurrentSlice();
          position[info.orientation] = info.sliceCoord;
        }
        mp->mpCursorWidget->setPosition(position);
        mp->mpCursorWidget->On();
      }
      else
      {
        mp->mpCursorWidget->Off();
      }
      zoomToFit();
    }

    /**
    */
    void SliceRenderGroup::setSegmentationImage(vtkImageData* pImage)
    {
      mp->mpLabelImage = pImage;
      mp->mpTransparencyMap->SetInputData(mp->mpLabelImage);
      mp->mpLabelMapper->Modified();
      showSegmentation(true);
    }

    /**
    */
    void SliceRenderGroup::showSegmentation(bool b)
    {
      mp->mpLabelActor->SetVisibility(b);
      mp->mpLabelActor->Modified();
    }

    /**
    */
    vtkRenderer* SliceRenderGroup::getRenderer()
    {
      return mp->mpRenderer;
    }

    /**
    */
    vtkRenderWindow* SliceRenderGroup::getRenderWindow()
    {
      return mp->mpRenderWindow;
    }

    /**
    */
    vtkRenderWindowInteractor* SliceRenderGroup::getInteractor()
    {
      return mp->mpRenderWindow->GetInteractor();
    }

    /**
    */
    void SliceRenderGroup::zoomToFit()
    {
      mp->mpInteractorStyle->zoomToFit();
      mp->mpInteractor->Render();
    }

    /**
    */
    SliceInfo SliceRenderGroup::getCurrentSlice() const
    {
      SliceInfo info;
      info.sliceNumber = mp->mpImageMapper->GetSliceNumber();
      info.orientation = mp->mpImageMapper->GetOrientation();
      auto i = info.orientation;
      info.sliceCoord  = mp->mpImage->GetOrigin()[i] + info.sliceNumber*mp->mpImage->GetSpacing()[i];   
      info.maxSlices   = mp->mpImageMapper->GetSliceNumberMaxValue();
      auto* plane      = mp->mpImageMapper->GetSlicePlane();
      plane->GetNormal(info.normal);
      plane->GetOrigin(info.origin);
      return info;
    }

    /**
    */
    void SliceRenderGroup::setCurrentSlice(double position[3])
    {
      // abort if out of range
      double bounds[6];
      mp->mpImage->GetBounds(bounds);
      for (int i(0); i<3; ++i)
      {
        if (bounds[2 * i] > position[i] || bounds[2 * i + 1] < position[i])
          return;
      }
      // set focal point to position, mapper property FocalPointOnSlice manages correct slice
      auto* camera = mp->mpRenderer->GetActiveCamera();
      double p[3];
      double f[3];
      camera->GetPosition(p);
      camera->GetFocalPoint(f);
      for (int i(0); i < 3; ++i)
      {
        if (i == mp->mpImageMapper->GetOrientation())
        {
          f[i] = position[i];
        }
      }
      camera->SetFocalPoint(f);
      camera->SetPosition(p);
    }

    /**
    */
    void SliceRenderGroup::setLookupTable(const std::vector<SegmentationLabel>& labels)
    {
      if (labels.empty())
        return;

      mp->mpLookupTable = make_vtk<vtkLookupTable>();
      mp->mpLookupTable->SetNumberOfTableValues(labels.size());
      auto maxValue = std::max_element(labels.begin(), labels.end(), [&] (const auto& lhs, const auto& rhs)
      {
        return lhs.grayValue < rhs.grayValue;
      });
      auto minValue = std::min_element(labels.begin(), labels.end(), [&] (const auto& lhs, const auto& rhs)
      {
        return lhs.grayValue < rhs.grayValue;
      });
      mp->mpLookupTable->SetRange(minValue->grayValue, maxValue->grayValue);

      for (const auto& label : labels)
      {
        double opacity = label.grayValue == 0 ? 0 : 0.5;
        mp->mpLookupTable->SetTableValue(label.grayValue, label.color.x(), label.color.y(), label.color.z(), opacity);
      }
      mp->mpLookupTable->Build();
      mp->mpTransparencyMap->SetLookupTable(mp->mpLookupTable);
      mp->mpTransparencyMap->Modified();
      mp->mpLabelMapper->Modified();
      mp->mpLabelActor->Modified();
    }
  }
}