#include "private/muk.pch"
#include "meshToBinaryImage.h"

#include "MukCommon/MukVector.h"
#include "MukCommon/vtk_tools.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkImageFileWriter.h>
#include <itkImageDuplicator.h>
#include <itkVTKImageToImageFilter.h>
#include <itkImageToVTKImageFilter.h>

#include <vtkImageData.h>
#include <vtkImageStencil.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataToImageStencil.h>

namespace gris
{
namespace muk
{
  /** \brief creates an image of the same size as #referenceImage and labels the pixels occupied by the mesh

    from https://www.vtk.org/Wiki/VTK/Examples/Cxx/PolyData/PolyDataToImageData
  */
  ImageInt3D::Pointer meshToBinaryImage(const ImageInt3D& referenceImage_, const vtkPolyData& mesh_, MukPixel label)
  {
    const auto* referenceImage = &referenceImage_;
    auto*       mesh = const_cast<vtkPolyData*>(&mesh_);
    ImageInt3D::Pointer result;
    {
      auto spacing = Vec3d(referenceImage->GetSpacing()[0], referenceImage->GetSpacing()[1], referenceImage->GetSpacing()[2]);
      auto origin  = Vec3d(referenceImage->GetOrigin()[0], referenceImage->GetOrigin()[1], referenceImage->GetOrigin()[2]);
      auto region  = referenceImage->GetLargestPossibleRegion();
      auto dim     = Vec3i(region.GetSize()[0], region.GetSize()[1], region.GetSize()[2]);

      auto whiteFilter = make_itk<itk::ImageToVTKImageFilter<ImageInt3D>>();
      whiteFilter->SetInput(referenceImage);
      whiteFilter->Update();

      // for whatever reason ImageToVTKImageFilter's input and output are the same
      // because we do not want to change the input we have to make a deep copy here
      auto whiteImage = make_vtk<vtkImageData>();
      whiteImage->DeepCopy(whiteFilter->GetOutput());

      // fill the image with background voxels:
      unsigned char inval = label;
      unsigned char outval = 0;
      auto count = whiteImage->GetNumberOfPoints();
      for (vtkIdType i = 0; i < count; ++i)
      {
        whiteImage->GetPointData()->GetScalars()->SetTuple1(i, label);
      }
      whiteImage->Modified();

      // polygonal data --> image stencil:
      auto pol2stenc = make_vtk<vtkPolyDataToImageStencil>();
      pol2stenc->SetInputData(mesh);
      pol2stenc->SetOutputOrigin(origin.data());
      pol2stenc->SetOutputSpacing(spacing.data());
      pol2stenc->SetOutputWholeExtent(whiteImage->GetExtent());
      pol2stenc->Update();

      // cut the corresponding white image and set the background:
      auto imgstenc = make_vtk<vtkImageStencil>();
      imgstenc->SetInputData(whiteImage);
      imgstenc->SetStencilData(pol2stenc->GetOutput());
      imgstenc->ReverseStencilOff();
      imgstenc->SetBackgroundValue(0);
      imgstenc->Update();

      auto converter = make_itk<itk::VTKImageToImageFilter<ImageInt3D>>();
      converter->SetInput(imgstenc->GetOutput());
      converter->Update();

      auto dupl = make_itk<itk::ImageDuplicator<ImageInt3D>>();
      dupl->SetInputImage(converter->GetOutput());
      dupl->Update();

      result = dupl->GetOutput();
    }
    return result;
  }

}
}