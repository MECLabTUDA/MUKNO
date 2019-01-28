#include "private/muk.pch"
#include "MedicalMultiViewWidget.h"
#include "SliceWidget.h"
#include "VolumeWidget.h"
#include "VtkWindow.h"
#include "Cursor3DSynchronizer.h"

#include "MukVisualization/SliceInteractorStyle.h"
#include "MukVisualization/SliceRenderGroup.h"
#include "MukVisualization/Cursor3DWidget.h"
#include "MukVisualization/ROISynchronizer.h"
#include "MukVisualization/ROIWidget.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukVector.h"

#include <vtkCamera.h>
#include <vtkRenderWindowInteractor.h>

#include <qlayout.h>

namespace
{
  enum EnQtViewPort
  {
    enUL, // upper left
    enUR,
    enLL,
    enLR,
    enNone,
  };
}

namespace gris
{
namespace muk
{
  /**
  */
  struct MedicalMultiViewWidget::Impl
  {
    Impl();
    ~Impl() {}

    void initialize(QWidget* parent);
    void toggleFullScreen3D(EnQtViewPort enPort);
    void takeScreenShot(EnQtViewPort enPort);

    QGridLayout*  mpLayout;
    VolumeWidget  m3DWindow;

    std::array<SliceWidget*,3>                         mSliceWidgets;
    //std::array<vtkSmartPointer<ROIWidget>,3>           mRois;
    //vtkSmartPointer<ROISynchronizer>                   mpRoiSync;
    vtkSmartPointer<Cursor3DSynchronizer>              mpCursorSync;
    vtkSmartPointer<vtkRenderWindowInteractor>         m3DInteractor;

    bool          mIsFullScreen = false;
  };

  /**
  */
  MedicalMultiViewWidget::Impl::Impl()
  {
  }

  /**
  */
  void MedicalMultiViewWidget::Impl::initialize(QWidget* parent)
  {
    mpLayout = new QGridLayout(parent);
    mpLayout->setContentsMargins(0, 0, 0, 0); // makes sure the Widget takes space of the whole parent widget
    mpLayout->addWidget(&m3DWindow, 1, 0);

    for (int i(0); i < mSliceWidgets.size(); ++i)
      mSliceWidgets[i] = new SliceWidget(parent);

    mpLayout->addLayout(mSliceWidgets[0]->getLayout(), 0, 0);
    mpLayout->addLayout(mSliceWidgets[1]->getLayout(), 0, 1);
    mpLayout->addLayout(mSliceWidgets[2]->getLayout(), 1, 1);
    auto& g1 = mSliceWidgets[0]->getRenderGroup();
    auto& g2 = mSliceWidgets[1]->getRenderGroup();
    auto& g3 = mSliceWidgets[2]->getRenderGroup();
    g1.setOrientation(enAxial);
    g2.setOrientation(enSagittal);
    g3.setOrientation(enCoronal);
    g1.getRenderWindow()->Render();
    g2.getRenderWindow()->Render();
    g3.getRenderWindow()->Render();

    for (size_t i(0); i < mSliceWidgets.size(); ++i)
    {
      auto cb = std::bind(&SliceWidget::setValue, mSliceWidgets[i], std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
      auto in = dynamic_cast<SliceInteractorStyle*>(mSliceWidgets[i]->getRenderGroup().getInteractor()->GetInteractorStyle());
      in->setCallbackPixelValue(cb);
    }

    //for (size_t i(0); i < mSliceWidgets.size(); ++i)
    //{
    //  //mRois[i]   = make_vtk<ROIWidget>();
    //}
    //mpRoiSync = make_vtk<ROISynchronizer>();
    mpCursorSync = make_vtk<Cursor3DSynchronizer>();
    //mpRoiSync->initialize(mBoxes[0], mBoxes[1], mBoxes[2]);
    mpCursorSync->initialize(g1.getCursor(), g2.getCursor(), g3.getCursor());
    mpCursorSync->initialize(g1,g2,g3);
    //mpCursorSync->initialize(mRois[0], mRois[1], mRois[2]);
    mpCursorSync->initialize(mSliceWidgets[0], mSliceWidgets[1], mSliceWidgets[2]);
  }

  /**
  */
  void MedicalMultiViewWidget::Impl::toggleFullScreen3D(EnQtViewPort enPort)
  {
    if (mIsFullScreen)
    {
      // enforce default grid layout
      mIsFullScreen = false;
      m3DWindow.hide(false);
      mSliceWidgets[0]->   hide(false);
      mSliceWidgets[1]->hide(false);
      mSliceWidgets[2]-> hide(false);
    }
    else
    {
      // hide others
      if (enPort != enUL)
        mSliceWidgets[0]->hide(true);
      if (enPort != enUR)
        mSliceWidgets[1]->hide(true);
      if (enPort != enLL)
        m3DWindow.hide(true);
      if (enPort != enLR)
        mSliceWidgets[2]->hide(true);
      mIsFullScreen = true;
    }
  }

  /**
  */
  void MedicalMultiViewWidget::Impl::takeScreenShot(EnQtViewPort enPort)
  {
    if (enPort == enLR)
    {
      m3DWindow.takeScreenShot();
    }
  }

  // =========================================================================================================
  
  /**
  */
  MedicalMultiViewWidget::MedicalMultiViewWidget(QWidget* parent)
    : QWidget(parent)
    , mp(std::make_unique<Impl>())
  {
    {
      QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      sizePolicy.setHorizontalStretch(5);
      sizePolicy.setVerticalStretch(0);
      setSizePolicy(sizePolicy);
    }
    mp->initialize(this);

    connect(mp->m3DWindow.getFullScreenButton(), &QPushButton::clicked, [this] () { mp->toggleFullScreen3D(enLL); });
    connect(mp->mSliceWidgets[0]->getFullScreenButton(), &QPushButton::clicked, [this] () { mp->toggleFullScreen3D(enUL); });
    connect(mp->mSliceWidgets[1]->getFullScreenButton(), &QPushButton::clicked, [this] () { mp->toggleFullScreen3D(enUR); });
    connect(mp->mSliceWidgets[2]->getFullScreenButton(), &QPushButton::clicked, [this] () { mp->toggleFullScreen3D(enLR); });

    connect(mp->m3DWindow.getScreenShotButton(),  &QPushButton::clicked, [this] () { mp->takeScreenShot(enLR); });
  }

  /**
  */
  MedicalMultiViewWidget::~MedicalMultiViewWidget()
  {
  }

  /**
  */
  VtkWindow* MedicalMultiViewWidget::get3DWindow() 
  { 
    return mp->m3DWindow.getVtkWindow();
  }

  /**
  */
  const VtkWindow* MedicalMultiViewWidget::get3DWindow() const 
  {
    return mp->m3DWindow.getVtkWindow();
  }

  /**
  */
  SliceWidget* MedicalMultiViewWidget::getAxialSliceWidget()
  {
    return mp->mSliceWidgets[0];
  }

  /**
  */
  SliceWidget* MedicalMultiViewWidget::getSagittalSliceWidget()
  {
    return mp->mSliceWidgets[1];
  }

  /**
  */
  SliceWidget* MedicalMultiViewWidget::getCoronalSliceWidget()
  {
    return mp->mSliceWidgets[2];
  }
}
}