#include "private/muk.pch"
#include "private/ImageOverlayCallback.h"

#include "MukCommon/muk_common.h"

#include <vtkExtractVOI.h>
#include <vtkImageData.h>
#include <vtkImageProperty.h>
#include <vtkImageReslice.h>
#include <vtkImageResliceMapper.h>
#include <vtkImageSlice.h>
#include <vtkObjectFactory.h>
#include <vtkPlane.h>
#include <vtkRenderer.h>
#include <vtkImplicitPlaneRepresentation.h>

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(ImageOverlayCallback);

  /**
  */
  ImageOverlayCallback::ImageOverlayCallback()
  {
    mpPlane = make_vtk<vtkPlane>();

    // the pipeline
    mpCropper = make_vtk<vtkExtractVOI>();  // class to focus on a small part of a larger data-cube, effectively cropping it

    mpReslice = make_vtk<vtkImageReslice>();
    mpReslice->SetOutputDimensionality(3);
    mpReslice->SetInterpolationModeToLinear();
    mpReslice->SetInputData(mpCropper->GetOutput());

    mpResliceMapper = make_vtk<vtkImageResliceMapper>();
    mpResliceMapper->SliceFacesCameraOff();
    mpResliceMapper->SliceAtFocalPointOff();
    mpResliceMapper->BorderOn();
    mpResliceMapper->SetInputConnection(mpCropper->GetOutputPort());

    mpIP = make_vtk<vtkImageProperty>();
    mpIP->SetColorWindow(2000);
    mpIP->SetColorLevel(1000);
    mpIP->SetAmbient(0.0);
    mpIP->SetDiffuse(1.0);
    mpIP->SetOpacity(1.0);
    mpIP->SetInterpolationTypeToLinear();

    mpSlice = make_vtk<vtkImageSlice>();
    mpSlice->SetProperty(mpIP);
    mpSlice->SetMapper(mpResliceMapper);
  }

  /**
  */
  ImageOverlayCallback::~ImageOverlayCallback()
  {
    if (mpRenderer)
      mpRenderer->RemoveViewProp(mpSlice);
  }

  /**
  */
  void ImageOverlayCallback::setOpacity(double v)
  {
    mpSlice->GetProperty()->SetOpacity(v);
  }

  /**
  */
  void ImageOverlayCallback::setRenderer(vtkRenderer* pObj) 
  {
    mpRenderer = pObj;
    mpRenderer->AddViewProp(mpSlice);
  }

  /**
  */
  void ImageOverlayCallback::setImage(vtkImageData* pObj) 
  {
    mpVolumeImage = pObj;
    mpCropper->SetInputData(mpVolumeImage);
  }
 
  /**
  */
  void ImageOverlayCallback::setRepresentation(vtkImplicitPlaneRepresentation* pRep) 
  {
    mpRep = pRep;
    mpRep->SetVisibility(true);
  }

  /**
    // ManualUpdate hast du gemacht (wobei das auch nur alles von execute verschoben ist)
  */
  void ImageOverlayCallback::modified()
  {
    mpRep->UpdatePlacement();
    mpRep->GetPlane(mpPlane);
    mpResliceMapper->SetSlicePlane(mpPlane);
    mpResliceMapper->Modified();
    mpSlice->Modified();
  }

  /**
    sets the vibility of the CT-Slice
  */
  void ImageOverlayCallback::setVisibility(bool b)
  {
    mpSlice->SetVisibility(b);
  }

  /**
  sets the vibility of the CT-Slice
  */
  bool ImageOverlayCallback::getVisibility()
  {
    return mpSlice->GetVisibility();
  }

  /**
  */
  void ImageOverlayCallback::Execute(vtkObject *caller, unsigned long eventId, void *callData)
  {
    modified();
  }

  /**
  */
  void ImageOverlayCallback::setVOI(const Vec3i& min, const Vec3i& max)
  {
    mpCropper->SetVOI(min.x(), max.x(), min.y(), max.y(), min.z(), max.z());
    mpCropper->Modified();
  }

  /**
  */
  vtkImageData* ImageOverlayCallback::getImage() const
  {
    return mpVolumeImage;
  }
}
}