#pragma once

#include "muk_qt_api.h"

#include "MukCommon/MukVector.h"
#include "MukCommon/muk_common.h"

#include <QWidget>
class QVTKWidget;
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>

#include <memory>

namespace gris
{
  namespace muk
  {

    class MUK_QT_API SphereInteraction : public QWidget
    {
      Q_OBJECT

      public:
        SphereInteraction(QWidget* widget);
        ~SphereInteraction();

      signals:
        void directionChanged();

      public:
        void initialize();
        void render();

      public:
        void  changeDirection(const Vec3d& direction);
        void  setDirection(const Vec3d& direction);
        Vec3d getDirection() const;

      private:
        struct ArrowMaker;

      private:
        QVTKWidget*                           widget3D;        
        vtkSmartPointer<vtkRenderWindow>              mpRenderWindow;
        vtkSmartPointer<vtkRenderer>                  mpRenderer;
        vtkSmartPointer<vtkOrientationMarkerWidget>   markerWidget;
        std::unique_ptr<ArrowMaker>           mpArrowMaker;
        double                                mCameraDistance;
    };

  }
}