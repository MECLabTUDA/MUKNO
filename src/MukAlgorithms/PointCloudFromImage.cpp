#include "private/muk.pch"
#include "PointCloudFromImage.h"

#include "AlgorithmFactory.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include "MukImaging/muk_imaging_tools.h"

#include <itkImageRegionConstIterator.h>

#include <vtkPolyData.h>

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(PointCloudFromImage);

    /**
    */
    struct PointCloudFromImage::Impl
    {
      vtkSmartPointer<vtkPolyData> output = nullptr;
      ImageInt3D*                    input  = nullptr;
      MukPixel  label = 1;
    };

    /**
    */
    PointCloudFromImage::PointCloudFromImage()
      : mp(std::make_unique<Impl>())
    {
      mp->output = make_vtk<vtkPolyData>();
      mDspType   = enDisplayPolyData;

      mInputPortTypes.push_back(enImageInt3D);
      mOutputPortTypes.push_back(enVtkPolyData);

      declareProperty<MukPixel>("LabelValue",  
        [=] (const auto val) { mp->label = val; },
        [=] ()               { return mp->label; });
    }

    /**
    */
    void PointCloudFromImage::setInput(unsigned int portId, void* pDataType)
    {
      mp->input = toDataType<ImageInt3D>(pDataType);
    }

    /**
    */
    void* PointCloudFromImage::getOutput(unsigned int portId)
    {
      return mp->output;
    }

    /**
    */
    void PointCloudFromImage::update()
    {
      auto points   = make_vtk<vtkPoints>();
      auto vertices = make_vtk<vtkCellArray>();
      auto iter = itk::ImageRegionConstIterator<ImageInt3D>(mp->input, mp->input->GetBufferedRegion());
      const auto cacheValue = mp->label;
      while ( ! iter.IsAtEnd())
      {
        if (iter.Get() == cacheValue)
        {
          itk::Point<double, 3> p;
          mp->input->TransformIndexToPhysicalPoint(iter.GetIndex(), p);
          vtkIdType pid[1];
          pid[0] = points->InsertNextPoint(p.GetDataPointer());
          vertices->InsertNextCell(1, pid);
        }
        ++iter;
      }
      mp->output->SetPoints(points);
      mp->output->SetVerts(vertices);
      mp->output->Modified();
    }
  }
}