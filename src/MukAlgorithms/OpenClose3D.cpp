#include "private/muk.pch"
#include "OpenClose3D.h"
#include "AlgorithmFactory.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkImageToVtkImageFilter.h>
#include <itkVtkImagetoImageFilter.h>

#include <vtkImageCast.h>
#include <vtkImageOpenClose3D.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(OpenClose3D)

  using ToItk = itk::VTKImageToImageFilter<ImageInt3D>;
  using ToVtk = itk::ImageToVTKImageFilter<ImageInt3D>;

  struct OpenClose3D::Impl
  {
    Impl()
    {
      toVtk  = make_itk<ToVtk>();
      filter = make_vtk<vtkImageOpenClose3D>();
      cast   = make_vtk<vtkImageCast>();
      toItk  = make_itk<ToItk>();
      cast->SetOutputScalarTypeToUnsignedShort();
    }

    void setKernelSize(const Vec3i& v)
    {
      kernelSize = v;
      filter->SetKernelSize(v.x(), v.y(), v.z());
    }

    itk::SmartPointer<ToVtk> toVtk;
    vtkSmartPointer<vtkImageOpenClose3D> filter;
    vtkSmartPointer<vtkImageCast> cast;
    itk::SmartPointer<ToItk> toItk;
    Vec3i kernelSize;
  };

  /**
  */
  OpenClose3D::OpenClose3D()
    : mp( std::make_unique<Impl>() )
  {
    mDspType = enDisplayOverlay3D;
    mInputPortTypes.push_back(enImageInt3D);
    mOutputPortTypes.push_back(enImageInt3D);
      
    declareProperty<double>("CloseValue",
      [=] (const double& d)   { mp->filter->SetCloseValue(d); mp->filter->Modified(); },
      [=] ()                  { return mp->filter->GetCloseValue(); });
    declareProperty<double>("OpenValue",
      [=] (const double& d)  { mp->filter->SetOpenValue(d); mp->filter->Modified(); },
      [=] ()                 { return mp->filter->GetOpenValue(); });
    declareProperty<Vec3i>("KernelSize",
      [=] (const Vec3i& v)  { mp->setKernelSize(v); },
      [=] ()                { return mp->kernelSize; });
  }

  /** \brief Explicit rebuilding of the pipline.

    A single modification of the toVtk-filter's input does not work.
    The OpenClose3D implementation is shit. This workaround is the first, that worked.
  */
  void OpenClose3D::setInput(unsigned int portId, void* pDataType)
  {
    auto* pData = toDataType<ImageInt3D>(pDataType);
    
    mp->toVtk->SetInput(pData);
    mp->filter->SetInputData(mp->toVtk->GetOutput());
    mp->cast->SetInputData(mp->filter->GetOutput());
    mp->toItk->SetInput(mp->cast->GetOutput());
    
    mp->toVtk ->Modified();
    mp->filter->Modified();
    mp->cast  ->Modified();
    mp->toItk ->Modified();
  }

  /**
  */
  void* OpenClose3D::getOutput(unsigned int portId)
  {
    return static_cast<void*>(mp->toItk->GetOutput());
  }

  /**
  */
  void OpenClose3D::update()
  {
    mp->toVtk->Update();
    mp->filter->Update();
    mp->cast->Update();
    mp->toItk->Update();
  }
}
}