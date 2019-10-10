#include "private/muk.pch"
#include "MarchingCubes.h"
#include "AlgorithmFactory.h"
#include "Remesher.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include "MukImaging/muk_imaging_tools.h"


#include <vtkMarchingCubes.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(MarchingCubes);

  /**
  */
  struct MarchingCubes::Impl
  {
    Impl()
      : mSurfaceValue(1.0)
      , mRemeshingValue(0)
      , mModified(false)
    {
      mpTrafo = make_itk<Trafo>();
      mpCubes = make_vtk<vtkMarchingCubes>();
      // atm: allow only one contour
      mOutput = make_vtk<vtkPolyData>();
    }

    void update()
    {
      mpTrafo->UpdateLargestPossibleRegion();
      mpCubes = make_vtk<vtkMarchingCubes>();
      mpCubes->SetNumberOfContours(1);
      mpCubes->SetValue(0, mSurfaceValue);
      mpCubes->SetInputData(mpTrafo->GetOutput());
      mpCubes->Update();

      if (mRemeshingValue)
      {
        auto pRemesh = std::make_unique<Remesher>();
        pRemesh->setInput(0, mpCubes->GetOutput());
        std::stringstream ss;
        ss << mRemeshingValue;
        pRemesh->setProperty("NumberOfSamples", ss.str());
        pRemesh->update();
        auto* mesh = static_cast<vtkPolyData*>(pRemesh->getOutput(0));
        mOutput->DeepCopy(mesh);
      }
      else
      {
        mOutput->DeepCopy(mpCubes->GetOutput());
      }
    }

    using Trafo = itk::ImageToVTKImageFilter<ImageInt3D>;

    ImageInt3D*    pInput = nullptr;
    Trafo::Pointer mpTrafo;
    vtkSmartPointer<vtkMarchingCubes> mpCubes;
    vtkSmartPointer<vtkPolyData>      mOutput;
    double        mModified;
    double        mSurfaceValue;
    unsigned int  mRemeshingValue; /// number of desired points on the contour
    itk::ModifiedTimeType mTime = 0;
  };

  /**
  */
  MarchingCubes::MarchingCubes()
    : mp(std::make_unique<Impl>())
  {
    mDspType = enDisplayPolyData;
    mInputPortTypes.push_back(enImageInt3D);
    mOutputPortTypes.push_back(enVtkPolyData);
    
    declareProperty<double>("Value",
      [=] (double v)  { mp->mSurfaceValue = v; mp->mModified = true; },
      [=] ()          { return mp->mSurfaceValue; });

    declareProperty<unsigned int>("Remeshing",
      [=] (double v)  { mp->mRemeshingValue = v; mp->mModified = true; },
      [=] ()          { return mp->mRemeshingValue; });
  }
  
  /**
  */
  void MarchingCubes::setInput(unsigned int portId, void* pDataType)
  {
    mp->pInput = static_cast<ImageInt3D*>(pDataType);
    mp->mpTrafo->SetInput( mp->pInput );
    mp->mModified = true;
  }

  void* MarchingCubes::getOutput(unsigned int portId)
  {
    return static_cast<void*>(mp->mOutput.GetPointer());
  }

  /**
  */
  void MarchingCubes::update()
  {
    auto in  = mp->pInput->GetMTime();
    auto out = mp->mTime;
    bool mTimeUnchanged = in < out;
    if (!mp->mModified && mTimeUnchanged)
      return;

    mp->mTime = mp->mpTrafo->GetInput()->GetMTime();
    mp->update();
    mp->mModified = false;
  }
}
}