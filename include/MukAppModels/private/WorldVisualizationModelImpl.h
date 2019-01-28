#pragma once
#include "WorldVisualizationModel.h"

#include "MukCommon/muk_common.h"

#include "MukQt/SliceWidget.h"
#include "MukQt/VtkWindow.h"

#include "MukVisualization/muk_colors.h"
#include "MukVisualization/SliceRenderGroup.h"
#include "MukVisualization/VisAbstractObject.h"

#include <vtkSmartPointer.h>

namespace gris
{
namespace muk
{
  /** \brief private implementation of WorldVisualizationModel
  */
  struct WorldVisualizationModel::Impl
  {
    Impl();
    void initialize();

    void setCtImage(vtkImageData* pImage);
    void setSegmentationImage(vtkImageData* pImage);

    VisAbstractObject* getSegmentationObject(const std::string& name);

    SliceWidget* mpAxialSliceWidget;
    SliceWidget* mpSagittalSliceWidget;
    SliceWidget* mpCoronalSliceWidget;
    VtkWindow*   mp3DWindow;

    SliceRenderGroup* mpAxialGroup;
    SliceRenderGroup* mpSagittalGroup;
    SliceRenderGroup* mpCoronalGroup;

    std::vector<SegmentationLabel> mLabels;

    std::vector<std::unique_ptr<VisAbstractObject>> mAlgorithmOutputs;
  };

  /**
  */
  WorldVisualizationModel::Impl::Impl()
    : mpAxialSliceWidget (nullptr)
    , mpSagittalSliceWidget(nullptr)
    , mpCoronalSliceWidget(nullptr)
    , mpAxialGroup(nullptr)
    , mpSagittalGroup(nullptr)
    , mpCoronalGroup(nullptr)
  {
    // setup base look up table (mukno labels go till 12
    mLabels.push_back( SegmentationLabel("Clear Label", 0, Vec3d(Colors::Black)) );
    mLabels.push_back( SegmentationLabel("InternalCarotidArtery", 1, Vec3d(Colors::Red)) );
    mLabels.push_back( SegmentationLabel("JugularVein", 2, Vec3d(Colors::Blue)) );
    mLabels.push_back( SegmentationLabel("FacialNerve", 3, Vec3d(Colors::LightYellow)) );
    mLabels.push_back( SegmentationLabel("Cochlea", 4, Vec3d(Colors::Green)) );
    mLabels.push_back( SegmentationLabel("ChordaTympani", 5, Vec3d(Colors::Cyan)) );
    mLabels.push_back( SegmentationLabel("Ossicles", 6, Vec3d(Colors::Purple)) );
    mLabels.push_back( SegmentationLabel("SemicircularCanals", 7, Vec3d(Colors::Orange)) );
    mLabels.push_back( SegmentationLabel("InternalAuditoryCanal", 8, Vec3d(Colors::Magenta)) );
    mLabels.push_back( SegmentationLabel("VestibularAqueduct", 9, Vec3d(Colors::Melon)) );
    mLabels.push_back( SegmentationLabel("ExternalAuditoryCanal", 10, Vec3d(Colors::Brown)) );
    mLabels.push_back( SegmentationLabel("Brain", 11, Vec3d(Colors::Brain)) );
    mLabels.push_back( SegmentationLabel("Skull", 12, Vec3d(Colors::Olive)) );
  }

  /**
  */
  void WorldVisualizationModel::Impl::setCtImage(vtkImageData* pImage)
  {
    mpAxialGroup->setImage(pImage);
    mpSagittalGroup->setImage(pImage);
    mpCoronalGroup->setImage(pImage);
  }

  /**
  */
  void WorldVisualizationModel::Impl::setSegmentationImage(vtkImageData* pImage)
  {
    mpAxialGroup->setSegmentationImage(pImage);
    mpSagittalGroup->setSegmentationImage(pImage);
    mpCoronalGroup->setSegmentationImage(pImage);
  }

  /**
  */
  void WorldVisualizationModel::Impl::initialize()
  {
    mpAxialGroup->setLookupTable(mLabels);
    mpSagittalGroup->setLookupTable(mLabels);
    mpCoronalGroup->setLookupTable(mLabels);
  }

  /** \brief fetch or create the object with the given name
  */
  VisAbstractObject* WorldVisualizationModel::Impl::getSegmentationObject(const std::string& name)
  {
    VisAbstractObject* result;
    auto iter = std::find_if(mAlgorithmOutputs.begin(), mAlgorithmOutputs.end(), [&] (const auto& obj) { return obj->getName() == name; } );
    if (iter == mAlgorithmOutputs.end())
    {
      mAlgorithmOutputs.push_back(std::make_unique<VisAbstractObject>(name));
      result = mAlgorithmOutputs.back().get();
      result->setRenderer(mp3DWindow->getRenderer());
      LOG_LINE << "add renderer";
    }
    else
    {
      result = iter->get();
    }
    return result;
  }
}
}
