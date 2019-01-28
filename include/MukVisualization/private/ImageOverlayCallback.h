#pragma once
#include "MukCommon/MukVector.h"

#include <vtkCommand.h>
#include <vtkSmartPointer.h>

class vtkExtractVOI;
class vtkImageData;
class vtkImageProperty;
class vtkImageReslice;
class vtkImageResliceMapper;
class vtkImageSlice;
class vtkImplicitPlaneRepresentation;
class vtkPlane;
class vtkRenderer;
class vtkTransform;

namespace gris
{
  namespace muk
  {
    /** \brief 

      Die Klasse haben wir ja zusammen nur aus dem Code eines deiner Mitarbeiter kopiert, also hab ich da ja eigentlich nix selber geschrieben.
      Deswegen nehm ich mir nicht heraus das zu kommentieren und so zu tun als hätte ich zu der Klasse irgendwas beigesteuert.
      Wenn du willst kann ich die aber noch kommentieren.

      Ja bitte :D
    */
    class ImageOverlayCallback : public vtkCommand
    {
      public:
        static ImageOverlayCallback* New();
        vtkTypeMacro(ImageOverlayCallback, vtkCommand);

      public:
        ImageOverlayCallback();
        ~ImageOverlayCallback();

      public:
        void setRenderer(vtkRenderer* pObj);
        void setImage(vtkImageData* pObj);
        void setRepresentation(vtkImplicitPlaneRepresentation* pRep);
        void modified();
        void setVisibility(bool b);
        bool getVisibility();
        void setOpacity(double d);
        void setVOI(const Vec3i& pMin, const Vec3i& pMax);

        vtkImageData* getImage() const;

      public:
        virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData);

      private:
        vtkSmartPointer<vtkImageReslice> mpReslice;
        vtkSmartPointer<vtkImageData> mpVolumeImage;
        vtkSmartPointer<vtkTransform> mpTransform;
        vtkSmartPointer<vtkPlane> mpPlane;
        vtkRenderer* mpRenderer;
        vtkSmartPointer<vtkImageResliceMapper> mpResliceMapper;
        vtkSmartPointer<vtkImageProperty> mpIP;
        vtkSmartPointer<vtkImageSlice> mpSlice;
        vtkSmartPointer<vtkExtractVOI> mpCropper;
        vtkImplicitPlaneRepresentation* mpRep;
    };
  }
}
