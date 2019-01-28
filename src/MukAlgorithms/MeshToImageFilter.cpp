#include "private/muk.pch"
#include "MeshToImageFilter.h"
#include "AlgorithmFactory.h"
#include "meshToBinaryImage.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkImageAlgorithm.h>

#include <vtkPolyData.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(MeshToImageFilter);

  /**
  */
  struct MeshToImageFilter::Impl
  {
    vtkPolyData* inputMesh = nullptr;
    ImageInt3D*    inputRefImage = nullptr;
    ImageInt3D::Pointer output;
    MukPixel label = 1;
  };

  /**
  */
  MeshToImageFilter::MeshToImageFilter()
    : mp(std::make_unique<Impl>())
  {
    mp->output = make_itk<ImageInt3D>();
    mDspType = enDisplayPolyData;

    mInputPortTypes.push_back(enVtkPolyData);
    mInputPortTypes.push_back(enImageInt3D);
    mOutputPortTypes.push_back(enImageInt3D);

    declareProperty<MukPixel>("LabelValue",  
      [=] (const auto val) { mp->label = val; },
      [=] ()               { return mp->label; });
  }

  /**
  */
  void MeshToImageFilter::setInput(unsigned int portId, void* pDataType)
  {
    switch (portId)
    {
      case 0:
        mp->inputMesh = toDataType<vtkPolyData>(pDataType);
      case 1:
        mp->inputRefImage = toDataType<ImageInt3D>(pDataType);
    };
  }

  /**
  */
  void* MeshToImageFilter::getOutput(unsigned int portId)
  {
    return mp->output;
  }

  /**
  */
  void MeshToImageFilter::update()
  {
    auto result = meshToBinaryImage(mp->inputRefImage, mp->inputMesh, mp->label);
    
    mp->output = ImageInt3D::New();
    // Allocate the image
    mp->output->CopyInformation( result );
    mp->output->SetRequestedRegion( result->GetRequestedRegion() );
    mp->output->SetBufferedRegion( result->GetBufferedRegion() );
    mp->output->Allocate();
    typename ImageInt3D::RegionType region = result->GetLargestPossibleRegion();
    itk::ImageAlgorithm::Copy(result.GetPointer(),mp->output.GetPointer(),region,region);
  }
}
}