#include "private/muk.pch"
#include "SmoothSurfaceExtraction.h"
#include "AlgorithmFactory.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukException.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkImageToVTKImageFilter.h>
#include <itkBinaryThresholdImageFilter.h>
#include "itkBinaryBallStructuringElement.h"
#include <itkBinaryMorphologicalClosingImageFilter.h>

#include <vtkImageThreshold.h>
#include <vtkImageOpenClose3D.h>
#include <vtkImageShrink3D.h>
#include <vtkImageGaussianSmooth.h>
#include <vtkImageConstantPad.h>
#include <vtkImageIslandRemoval2D.h>
#include <vtkPolyDataNormals.h>
#include <vtkImageData.h>
#include <vtkContourFilter.h>
#include <vtkImageResample.h>
#include <vtkTriangleFilter.h>

namespace
{
  using namespace gris::muk;

  using Element       = itk::BinaryBallStructuringElement<ImageInt3D::PixelType, ImageInt3D::ImageDimension>;
  using ClosingFilter = itk::BinaryMorphologicalClosingImageFilter <ImageInt3D, ImageInt3D, Element>;
  using ToItk         = itk::ImageToVTKImageFilter<ImageInt3D>();
}

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(SmoothSurfaceExtraction)

  /**
  */
  SmoothSurfaceExtraction::SmoothSurfaceExtraction()
    : mPaddingValue(0)
    , mSurfaceValue(127)
    , mLabelValue(1)
    , mAreaTreshold(0)
  {
    mDspType = enDisplayPolyData;
    mInputPortTypes.push_back(enImageInt3D);
    mOutputPortTypes.push_back(enVtkPolyData);

    mpOutputMesh = make_vtk<vtkPolyDataNormals>();
    mpOutputMesh->SplittingOff();

    declareProperty<MukPixel>("LabelValue", MUK_SET(double, setLabel), MUK_GET(getLabel));
    declareProperty<double>("SurfaceValue", MUK_SET(double, setSurfaceValue), MUK_GET(getSurfaceValue));
    declareProperty<Vec3d>("Sigma",         MUK_C_SET(Vec3d, setSigma), MUK_GET(getSigma));
    declareProperty<Vec3d>("ClosingKernel", MUK_C_SET(Vec3d, setClosingKernel), MUK_GET(getClosingKernel));
    declareProperty<int>("Padding",         MUK_SET(int, setPadding), MUK_GET(getPadding));
    declareProperty<int>("AreaThreshold",   MUK_SET(int, setAreaThreshold), MUK_GET(getAreaThreshold));
  }
  
  /**
  */
  void SmoothSurfaceExtraction::setInput(unsigned int portId, void* pDataType)
  {
    mpInputImage = toDataType<ImageInt3D>(pDataType);
  }

  /**
  */
  void* SmoothSurfaceExtraction::getOutput(unsigned int portId)
  {
    return mpOutputMesh->GetOutput();
  }
  
  /**
  */
  void SmoothSurfaceExtraction::update()
  {
    vtkImageAlgorithm* pCurrentPipelineEnd = nullptr;

    auto pThresh = make_itk<itk::BinaryThresholdImageFilter<ImageInt3D, ImageInt3D>>();
    pThresh->SetInput(mpInputImage);
    pThresh->SetLowerThreshold(mLabelValue);
    pThresh->SetUpperThreshold(mLabelValue);
    pThresh->SetOutsideValue(1);
    pThresh->Update();

    const bool useClosing  = mClosingKernel.x() > 0 && mClosingKernel.y() > 0 && mClosingKernel.z() > 0;
    using Element       = itk::BinaryBallStructuringElement<ImageInt3D::PixelType, ImageInt3D::ImageDimension>;
    using ClosingFilter = itk::BinaryMorphologicalClosingImageFilter <ImageInt3D, ImageInt3D, Element>;
    Element structuringElement;
    auto closingFilter = make_itk<ClosingFilter>();
    if (useClosing)
    {
      itk::Size<3u> size;
      size[0] = mClosingKernel.x();
      size[1] = mClosingKernel.y();
      size[2] = mClosingKernel.z();
      structuringElement.SetRadius(size);
      structuringElement.CreateStructuringElement();
      LOG_LINE << "use smoothing with " << mClosingKernel;
      closingFilter->SetInput(pThresh->GetOutput());
      closingFilter->SetKernel(structuringElement);
      closingFilter->Update();
    }

    auto pVtkImage = make_itk<itk::ImageToVTKImageFilter<ImageInt3D>>();
    if (useClosing)
      pVtkImage->SetInput(closingFilter->GetOutput());
    else
      pVtkImage->SetInput(pThresh->GetOutput());
    pVtkImage->Update();
    
    //// Map surface value from 1 to 255
    auto pThreshold = make_vtk<vtkImageThreshold>();
    pThreshold->SetInputData(pVtkImage->GetOutput());
    pThreshold->ThresholdBetween(1, 1);
    pThreshold->SetInValue(255.0);
    pThreshold->SetOutValue(0.0);
    pCurrentPipelineEnd = pThreshold;
    pCurrentPipelineEnd->Update();

    //auto pOpenClose = make_vtk<vtkImageOpenClose3D>();
    //if ( mClosingKernel.x() > 0 
    //  && mClosingKernel.y() > 0 
    //  && mClosingKernel.z() > 0) 
    //{
    //  // Set up open-close filer
    //  pOpenClose->SetOpenValue(0.0);
    //  pOpenClose->SetCloseValue(255.0);
    //  pOpenClose->SetKernelSize(mClosingKernel.x(), mClosingKernel.y(), mClosingKernel.z());
    //  pOpenClose->SetInputData(pCurrentPipelineEnd->GetOutput());
    //  pCurrentPipelineEnd = pOpenClose;
    //  pCurrentPipelineEnd->Update();
    //}

    // Set up island removal algorithm	
    auto pIslandRemoval = make_vtk<vtkImageIslandRemoval2D>();
    pIslandRemoval->SetIslandValue(0.0);
    pIslandRemoval->SetReplaceValue(255.0);
    pIslandRemoval->SetAreaThreshold(mAreaTreshold);
    pIslandRemoval->SetInputData(pCurrentPipelineEnd->GetOutput());
    pCurrentPipelineEnd = pIslandRemoval;
    pCurrentPipelineEnd->Update();

    // Pad image, if desired
    auto pConstantPad = make_vtk<vtkImageConstantPad>();
    if (mPaddingValue != 0)
    {
      int dims[3];
      pCurrentPipelineEnd->GetOutput()->GetDimensions(dims);
      int extent[6] = { -mPaddingValue, dims[0] + mPaddingValue, -mPaddingValue, dims[1] + mPaddingValue, -mPaddingValue, dims[2] + mPaddingValue };
      pConstantPad->SetOutputWholeExtent(extent);
      pConstantPad->SetInputData(pCurrentPipelineEnd->GetOutput());
      pCurrentPipelineEnd = pConstantPad;
      pCurrentPipelineEnd->Update();
    }

    // Smooth image
    auto pSmooth        = make_vtk<vtkImageGaussianSmooth>();
    auto pContourFilter = make_vtk<vtkContourFilter>();
    auto pTriangleFilter = make_vtk<vtkTriangleFilter>();
    pSmooth->SetInputData(pCurrentPipelineEnd->GetOutput());
    pSmooth->SetStandardDeviations(mSigma.data());
    pSmooth->Update();
    pContourFilter->SetInputData(pSmooth->GetOutput());
    pContourFilter->SetValue(0, mSurfaceValue);
    pContourFilter->Update();
    pTriangleFilter->SetInputData(pContourFilter->GetOutput());
    pTriangleFilter->Update();

    // Compute surface normals
    mpOutputMesh->SetInputData(pTriangleFilter->GetOutput());
    mpOutputMesh->Update();
    LOG_LINE << "remaining points " << mpOutputMesh->GetOutput()->GetNumberOfPoints();
  }
}
}

