#include "private/muk.pch"
#include "SliceInteractorStyle.h"

#include "vtkAbstractPropPicker.h"
#include "vtkActor2DCollection.h"
#include "vtkAssemblyPath.h"
#include "vtkCallbackCommand.h"
#include <vtkCamera.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkImageSliceMapper.h>
#include <vtkMapper.h>
#include <vtkObjectFactory.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <algorithm>

namespace
{
  const int VTKIS_PICK  = 1025;
  const int VTKIS_SLICE = 1026;
}

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(SliceInteractorStyle);

  SliceInteractorStyle::SliceInteractorStyle()
  {
    mXViewRightVector[0] = 0;
    mXViewRightVector[1] = 1;
    mXViewRightVector[2] = 0;

    mXViewUpVector[0] = 0;
    mXViewUpVector[1] = 0;
    mXViewUpVector[2] = 1;

    mYViewRightVector[0] = 1;
    mYViewRightVector[1] = 0;
    mYViewRightVector[2] = 0;

    mYViewUpVector[0] = 0;
    mYViewUpVector[1] = 0;
    mYViewUpVector[2] = 1;

    mZViewRightVector[0] = 1;
    mZViewRightVector[1] = 0;
    mZViewRightVector[2] = 0;

    mZViewUpVector[0] = 0;
    mZViewUpVector[1] = 1;
    mZViewUpVector[2] = 0;
  }

  /**
  */
  void SliceInteractorStyle::StartPick()
  {
    if (State != VTKIS_NONE)
    {
      return;
    }
    StartState(VTKIS_PICK);
    if (HandleObservers)
    {
      InvokeEvent(vtkCommand::StartPickEvent, this);
    }
  }

  /**
  */
  void SliceInteractorStyle::EndPick()
  {
    if (State != VTKIS_PICK)
    {
      return;
    }
    if (HandleObservers)
    {
      InvokeEvent(vtkCommand::EndPickEvent, this);
    }
    StopState();
  }

  //----------------------------------------------------------------------------
  void SliceInteractorStyle::OnMouseMove()
  {
    int x = Interactor->GetEventPosition()[0];
    int y = Interactor->GetEventPosition()[1];

    if (Interactor->GetControlKey() && mHasCallback)
    {
      double pos[3];
      Interactor->GetPicker()->Pick(Interactor->GetEventPosition()[0], Interactor->GetEventPosition()[1], 
        0,  // always zero.
        CurrentRenderer);
      Interactor->GetPicker()->GetPickPosition(pos);
      auto* img = getImage();
      if (img)
      {
        auto* s = img->GetSpacing();
        auto* o = img->GetOrigin();
        int idx[3];
        for (int i(0); i < 3; ++i)
          idx[i] = static_cast<int>( (pos[i]-o[i]) / s[i] + 0.5);
        
        mCallback(idx[0], idx[1], idx[2], img->GetScalarComponentAsDouble(idx[0], idx[1], idx[2], 0));
      }
    }

    switch (State)
    {
      case VTKIS_PICK:
        FindPokedRenderer(x, y);
        Pick();
        InvokeEvent(vtkCommand::InteractionEvent, nullptr);
        break;
      case VTKIS_SLICE:
        FindPokedRenderer(x, y);
        InvokeEvent(vtkCommand::InteractionEvent, nullptr);
        break;
    }

    // Call parent to handle all other states and perform additional work
    Superclass::OnMouseMove();
  }

  //----------------------------------------------------------------------------
  void SliceInteractorStyle::OnLeftButtonDown()
  {
    int x = Interactor->GetEventPosition()[0];
    int y = Interactor->GetEventPosition()[1];

    FindPokedRenderer(x, y);
    if (CurrentRenderer == nullptr)
    {
      return;
    }
    else
    {
      //Superclass::OnLeftButtonDown();
    }
  }

  /**
  */
  void SliceInteractorStyle::OnLeftButtonUp()
  {
    /*switch (State)
    {
      default:
        ;
    }*/
    // Call parent to handle all other states and perform additional work
    Superclass::OnLeftButtonUp();
  }

  /**
  */
  void SliceInteractorStyle::OnMouseWheelForward()
  {
    GrabFocus(EventCallbackCommand);
    const int number = Interactor->GetControlKey() ? 10 : 1;
    slice(number);
    if ( Interactor )
    {
      ReleaseFocus();
    }
  }


  /**
  */
  void SliceInteractorStyle::OnMouseWheelBackward()
  {
    GrabFocus(EventCallbackCommand);
    const int number = Interactor->GetControlKey() ? 10 : 1;
    slice(-number);
    if ( Interactor )
    {
      ReleaseFocus();
    }
  }

  //----------------------------------------------------------------------------
  void SliceInteractorStyle::OnMiddleButtonDown()
  {
    FindPokedRenderer(Interactor->GetEventPosition()[0], Interactor->GetEventPosition()[1]);

    if (CurrentRenderer == nullptr)
    {
      return;
    }
    Superclass::OnMiddleButtonDown();
  }

  //----------------------------------------------------------------------------
  void SliceInteractorStyle::OnMiddleButtonUp()
  {
    // Call parent to handle all other states and perform additional work
    Superclass::OnMiddleButtonUp();
  }

  //----------------------------------------------------------------------------
  void SliceInteractorStyle::OnRightButtonDown()
  {
    int x = Interactor->GetEventPosition()[0];
    int y = Interactor->GetEventPosition()[1];

    FindPokedRenderer(x, y);
    if (CurrentRenderer == nullptr)
    {
      return;
    }

    // Redefine this button + shift to handle pick
    GrabFocus(EventCallbackCommand);
    Superclass::OnRightButtonDown();
  }

  //----------------------------------------------------------------------------
  void SliceInteractorStyle::OnRightButtonUp()
  {
    // Call parent to handle all other states and perform additional work
    Superclass::OnRightButtonUp();
  }

  //----------------------------------------------------------------------------
  void SliceInteractorStyle::OnChar()
  {
    vtkRenderWindowInteractor *rwi = Interactor;

    switch (rwi->GetKeyCode())
    {
      case 'f' :
      case 'F' :
      {
        zoomToFit();
        break;
      }
      case 'g' :
      case 'G' :
      {
        AnimState = VTKIS_ANIM_ON;
        vtkAssemblyPath *path=nullptr;
        FindPokedRenderer(rwi->GetEventPosition()[0], rwi->GetEventPosition()[1]);
        rwi->GetPicker()->Pick(rwi->GetEventPosition()[0], rwi->GetEventPosition()[1], 0.0, CurrentRenderer);
        vtkAbstractPropPicker *picker;
        if ( (picker=vtkAbstractPropPicker::SafeDownCast(rwi->GetPicker())) )
        {
          path = picker->GetPath();
        }
        if ( path != nullptr )
        {
          rwi->FlyToImage(CurrentRenderer,picker->GetPickPosition());
        }
        AnimState = VTKIS_ANIM_OFF;
        break;
      }
      case 'r' :
      case 'R' :
      {
        // reset the camera
        Superclass::OnChar();
        break;
      }
      case 't' :
      case 'T' :
      {
        // turn upside down
        turnUpsideDown();
        Interactor->Render();
        break;
      }
      case 'x' :
      case 'X' :
      {
        setOrientationToX();
        Interactor->Render();
        break;
      }
      case 'y' :
      case 'Y' :
      {
        setOrientationToY();
        Interactor->Render();
        break;
      }
      case 'z' :
      case 'Z' :
      {
        setOrientationToZ();
        Interactor->Render();
        break;
      }
      default:
      {
        Superclass::OnChar();
        break;
      }
    }
  }

  /**
  */
  void SliceInteractorStyle::setOrientationToX()
  {
    setImageOrientation(mXViewRightVector, mXViewUpVector);
    Interactor->Render();
  }

  /**
  */
  void SliceInteractorStyle::setOrientationToY()
  {
    setImageOrientation(mYViewRightVector, mYViewUpVector);
    Interactor->Render();
  }

  /**
  */
  void SliceInteractorStyle::setOrientationToZ()
  {
    setImageOrientation(mZViewRightVector, mZViewUpVector);
    Interactor->Render();
  }

  /** \brief turn camera upside down
  */
  void SliceInteractorStyle::turnUpsideDown()
  {
    if (!CurrentRenderer)
      return;
    double upVec[3];
    double focalPoint[3];
    double position[3];
    auto* camera = CurrentRenderer->GetActiveCamera();
    camera->GetViewUp(upVec);    
    camera->GetFocalPoint(focalPoint);
    camera->GetPosition(position);
    for (int i(0); i < 3; ++i)
    {
      upVec[i] *= -1;
      position[i] += 2 * (focalPoint[i] - position[i]);
    }
    camera->SetViewUp(upVec);
    camera->SetPosition(position);
  }

  /**
  */
  void SliceInteractorStyle::setImageOrientation(const double leftToRight[3], const double viewUp[3])
  {
    if (CurrentRenderer)
    {
      // the cross product points out of the screen
      double vector[3];
      vtkMath::Cross(leftToRight, viewUp, vector);
      double focus[3];
      vtkCamera *camera = CurrentRenderer->GetActiveCamera();
      camera->GetFocalPoint(focus);
      double d = camera->GetDistance();
      camera->SetPosition(focus[0] + d*vector[0],
        focus[1] + d*vector[1],
        focus[2] + d*vector[2]);
      camera->SetFocalPoint(focus);
      camera->SetViewUp(viewUp);
    }
  }

  /**
  */
  void SliceInteractorStyle::Pick()
  {
    InvokeEvent(vtkCommand::PickEvent, this);
  }

  /**
  */
  void SliceInteractorStyle::OnEnter()
  {
    SetCurrentRenderer(Interactor->FindPokedRenderer(0,0));
  }

  /**
  */
  vtkImageData* SliceInteractorStyle::getImage()
  {
    if ( ! CurrentRenderer)
      return nullptr;

    auto* props = CurrentRenderer->GetViewProps();

    vtkImageData* pImage= nullptr;
    props->InitTraversal();
    vtkProp* prop = props->GetNextProp();
    int i(0);
    for (; prop != nullptr; prop = props->GetNextProp())
    {
      auto* actor =  dynamic_cast<vtkImageSlice*>(prop);
      if (! actor)
        continue;
      pImage = actor->GetMapper()->GetInput();
      if (pImage)
      {
        break;
      }
    }
    return pImage;
  }

  /**
  */
  vtkImageSliceMapper* SliceInteractorStyle::getMapper()
  {
    if ( ! CurrentRenderer)
      return nullptr;

    auto* props = CurrentRenderer->GetViewProps();

    vtkImageData* pImage= nullptr;
    vtkImageSliceMapper* pMapper= nullptr;
    props->InitTraversal();
    vtkProp* prop = props->GetNextProp();
    int i(0);
    for (; prop != nullptr; prop = props->GetNextProp())
    {
      auto* actor =  dynamic_cast<vtkImageSlice*>(prop);
      if (! actor)
        continue;
      pMapper = static_cast<vtkImageSliceMapper*>(actor->GetMapper());
      pImage  = actor->GetMapper()->GetInput();
      if (pImage)
      {
        break;
      }
    }
    return pMapper;
  }
  
  /**
  */
  SliceInteractorStyle::EnOrientationType SliceInteractorStyle::getOrientationType() const
  {
    double viewUp[3];
    CurrentRenderer->GetActiveCamera()->GetViewUp(viewUp);
    bool x, y, z;
    x = y = z = true;
    for (int i(0); i<3; ++i)
    {
      if (viewUp[i] != mXViewUpVector[i])
        x = false;
      if (viewUp[i] != mYViewUpVector[i])
        y = false;
      if (viewUp[i] != mZViewUpVector[i])
        z = false;
    }
    // one is always true
    if (x)
      return enXView;
    else if (y)
      return enYView;
    else
      return enZView;
  }

  /**
  */
  void SliceInteractorStyle::slice(int count)
  {
    vtkRenderWindowInteractor *rwi = Interactor;
    if (nullptr == rwi)
      return;

    auto* pImage = getImage();
    if (nullptr == pImage)
      return;

    // get necessary image parameters
    double spacing[3];
    double origin[3];
    int    extent[6];
    pImage->GetOrigin(origin);
    pImage->GetSpacing(spacing);
    pImage->GetExtent(extent);
    double bbox[6];
    for (int i(0); i<3; ++i)
    {
      bbox[2*i]   = origin[i] + extent[2*i]   * spacing[i];
      bbox[2*i+1] = origin[i] + extent[2*i+1] * spacing[i];
    }
    // get necessary camera parameters
    vtkCamera* camera = CurrentRenderer->GetActiveCamera();
    double normal[3];
    double focalPoint[3];
    double position[3];
    camera->GetViewPlaneNormal(normal);
    camera->GetFocalPoint(focalPoint);
    camera->GetPosition(position);
    // limit movement to x,y or z
    for (int i(0); i < 3; ++i)
      if (abs(normal[i]) < 10e-5) // == 0
        spacing[i] = 0;
    // calculate new paramters
    for (int i(0); i < 3; ++i)
    {
      focalPoint[i] += count*spacing[i]*normal[i];
      position[i]   += count*spacing[i]*normal[i];
    }
    // apply only if in range
    bool inRange = true;
    const double eps = 0; // some margin
    for (int i(0); i<3; ++i)
    {
      if (abs(normal[i]) > 10e-5 
        && 
         (focalPoint[i] < bbox[2*i] - eps || focalPoint[i] > bbox[2*i+1] + eps) )
      {
        inRange = false;
      }
    }
    if (inRange)
    {
      camera->SetFocalPoint(focalPoint);
      camera->SetPosition(position);
      // ugly hack to enforce staying on the center of voxels
      auto pMapper     = getMapper();
      pMapper->Update();
      auto sliceNumber = pMapper->GetSliceNumber();
      for (int i(0); i < 3; ++i)
      {
        if (normal[i])
        {
          const auto p = origin[i] + sliceNumber*spacing[i];
          const auto d = p - focalPoint[i];
          focalPoint[i] += d;
          position[i] += d;
        }
      }
      std::cout << std::endl;
      camera->SetFocalPoint(focalPoint);
      camera->SetPosition(position);
    }
    InvokeEvent(enSliceChangedEvent);
    rwi->Render();
  }

  /** \brief assures that the whole image fits either screen height or width

    computes world coordinates of display size (lower left corner and upper right corner).
    retrieves image width and height.
    then scales accordingly
  */
  void SliceInteractorStyle::zoomToFit()
  {
    if (CurrentRenderer == nullptr)
      return;
    auto* pImage = getImage();
    if (pImage == nullptr)
      return;
    // get image parameters
    double origin[3];
    int    extent[6];
    double spacing[3];
    pImage->GetSpacing(spacing);
    pImage->GetOrigin(origin);
    pImage->GetExtent(extent);
    // get camera parameters
    auto camera = CurrentRenderer->GetActiveCamera();
    double normal[3];
    double position[3];
    double focalPoint[3];
    camera->GetViewPlaneNormal(normal);
    camera->GetPosition(position);
    camera->GetFocalPoint(focalPoint);
    // position camera so it looks at the center of the current slice
    for (int i(0); i < 3; ++i)
    {
      if (abs(normal[i]) < 10e-5)
      {
        double center = origin[i] + 0.5 * spacing[i] * (extent[2*i] + extent[2*i+1]);
        double t = center - position[i];
        position[i]   += t;
        focalPoint[i] += t;
      }
    }
    camera->SetPosition(position);
    camera->SetFocalPoint(focalPoint);
    // --------------------------------------------------------------------
    // zoom so the image has the size of the viewport, use dolly for that
    double viewUp[3];
    camera->GetViewUp(viewUp);
    // first some basic parameters depending on the direction
    double imageWidth;
    double imageHeight;
    int idxNormal;
    int idxWidth;
    int idxHeight;
    // each if clause is true once
    for (int i(0); i<3;++i)
    {
      if (viewUp[i])
      {
        idxHeight = i;
        imageHeight  = (extent[2*i+1] - extent[2*i] + 1.0) * spacing[i];
      }
      else if (!normal[i])
      {
        idxWidth = i;
        imageWidth   = (extent[2*i+1] - extent[2*i] + 1.0) * spacing[i];
      }
      else
        idxNormal = i;
    }
    // now compute the scale of the dolly movement
    double displayFocalPoint[3];
    ComputeWorldToDisplay(focalPoint[0], focalPoint[1], focalPoint[2], displayFocalPoint);
    auto focalDepth = displayFocalPoint[2];
    double llc[4], urc[4];
    int* size = CurrentRenderer->GetSize();
    ComputeDisplayToWorld(0,       0,       focalDepth, llc);
    ComputeDisplayToWorld(size[0], size[1], focalDepth, urc);    
    auto   heightFrustum = urc[idxHeight]-llc[idxHeight];
    auto   widthFrustum  = urc[idxWidth] -llc[idxWidth];
    auto   heightFactor  = std::abs(heightFrustum/imageHeight);
    double widthFactor   = std::abs(widthFrustum /imageWidth);
    // perform the changes on the camera
    Dolly( std::min(heightFactor, widthFactor) );    
    Interactor->Render();
  }
}
}