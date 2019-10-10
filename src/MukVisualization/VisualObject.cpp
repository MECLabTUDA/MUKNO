#include "private/muk.pch"
#include "VisualObject.h"
#include "muk_colors.h"

#include "PolyDataHandler.h"
#include "PolyDataMapperHandler.h"

#include "MukCommon/MukException.h"
#include "MukCommon/overload_deduction.h"
#include "MukCommon/vtk_tools.h"

#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>

#include <boost/format.hpp>

using namespace gris::muk;


namespace gris
{
  namespace muk
  {
    /**
    */
    VisualObject::VisualObject()
      : mpActor( make_vtk<vtkActor>())
      , mpMapper(make_vtk<vtkPolyDataMapper>())
      , mpData(make_vtk<vtkPolyData>())
      , mpRenderer(nullptr)
      , mProp(nullptr)
      , mDefaultColor(Colors::Gray)
    {
      mpMapper->SetInputData(mpData);
      mpActor->SetMapper(mpMapper);
      mpActor->GetProperty()->SetPointSize(5);

      declareProperties();      
    }
    
    /**
    */
    VisualObject::VisualObject(const gstd::DynamicProperty& prop)
      : mProp(&prop)
      , mpActor (make_vtk<vtkActor>())
      , mpMapper(make_vtk<vtkPolyDataMapper>())
      , mpData  (make_vtk<vtkPolyData>())
      , mpRenderer(nullptr)
      , mDefaultColor(Colors::Gray)
    {      
      mpMapper->SetInputData(mpData);
      mpActor->SetMapper(mpMapper);
      mpActor->GetProperty()->SetPointSize(5);

      declareProperties();      
    }

    /**
    */
    VisualObject::~VisualObject()
    {
      if (mpRenderer)
      {
        mpRenderer->RemoveActor(mpActor);
        std::for_each(mpActors.begin(), mpActors.end(), [&] (auto& actor) { mpRenderer->RemoveActor(actor); } );
      }
    }

    /**
    */
    void VisualObject::declareProperties()
    {
      declareProperty<bool>("Visibility",
        [=] (bool b) { setVisibility(b); },
        [=] ()       { return getVisibility(); });
      declareProperty<double>("Opacity",
        [=] (double d) { setOpacity(d); },
        [=] ()         { return getOpacity(); });
      declareProperty<Vec3d>("Color",
        [&] (const Vec3d& color) { setDefaultColor(color); setColors(getDefaultColor()); },
        [&] ()                   { return getDefaultColor(); });
    }
        
    /**
    */
    void VisualObject::setRenderer(vtkRenderer* pRenderer)
    {
      mpRenderer = pRenderer;
      if (pRenderer)
      {
        pRenderer->AddActor(mpActor);
        std::for_each(mpActors.begin(), mpActors.end(), [&] (auto& actor) { pRenderer->AddActor(actor); });
      }
    }

    /**
    */
    void VisualObject::setData(vtkSmartPointer<vtkPolyData> data)
    {
      if (data->GetNumberOfPoints() != mpData->GetNumberOfPoints())
      {
        setColors(mDefaultColor);
      }
      mpData = data;
      mpMapper->SetInputData(mpData);
      if (nullptr != mpData->GetPointData()->GetScalars("Colors"))
      {
        mpMapper->ScalarVisibilityOn();
      }
      mpMapper->Modified();
    }

    /**
    */
    vtkSmartPointer<vtkPolyData> VisualObject::getData()
    {
      return mpData;
    }

    /**
    */
    void VisualObject::setDefaultColor(double color[3])
    {
      setDefaultColor(Vec3d(color[0], color[1], color[2]));
    }

    /**
    */
    void VisualObject::setDefaultColor(const Vec3d& color)
    {
      mDefaultColor = color;
    }

    /**
    */
    const Vec3d& VisualObject::getDefaultColor() const
    {
      return mDefaultColor;
    }

    /**
    */
    void VisualObject::setColors(double color[3])
    {
      Vec3d c(color);
      setColors(c);
    }

    /**
    */
    void VisualObject::setColors(const Vec3d& color)
    {
      double d[3] = {color.x(), color.y(), color.z()};
      mpActor->GetProperty()->SetColor(d);
      mpMapper->ScalarVisibilityOff();
      mpMapper->Modified();
    }

    /**
    */
    void VisualObject::setColors(const std::vector<Vec3d>& colors)
    {
      if (mpData.Get() == nullptr)
        return;

      auto colorArray = make_vtk<vtkUnsignedCharArray>();
      colorArray->SetName("Colors");
      colorArray->SetNumberOfComponents(3);

      const size_t N = mpData->GetNumberOfPoints();
      if (N != colors.size())
      {
        throw MUK_EXCEPTION_SIMPLE( (boost::format("size of color-array (%d) does not match size of internal data (%d)!") % colors.size() % N).str().c_str() );
      }

      for (const auto& c : colors)
      {
        char col[3] = {static_cast<char>(255 * c.x()), static_cast<char>(255 * c.y()), static_cast<char>(255 * c.z()) };
        colorArray->InsertNextTuple3(col[0], col[1], col[2]);
      }
      auto pPointData = mpData->GetPointData();
      if (pPointData->HasArray("Colors"))
      {
        auto N = pPointData->GetNumberOfArrays();
        for(vtkIdType i(0); i<N; ++i)
        {
          std::string name = pPointData->GetArrayName(i);
          if (name == "Colors")
          {
            pPointData->RemoveArray(i);
          }
        }
      }
      mpData->GetPointData()->AddArray(colorArray);
      mpMapper->SelectColorArray("Colors");
      mpMapper->SetScalarModeToUsePointFieldData();
      mpMapper->SetColorModeToDefault();
      mpMapper->ScalarVisibilityOn();
      mpMapper->Modified();
    }

    /**
    */
    void VisualObject::setVisibility(bool b)
    {
      mpActor->SetVisibility(b);
      mpActor->Modified();
      std::for_each(mpActors.begin(), mpActors.end(), [&] (auto& actor) { actor->SetVisibility(b); actor->Modified(); });
    }

    /**
    */
    void VisualObject::setOpacity(double val)
    {
      mpActor->GetProperty()->SetOpacity(val);
      mpActor->Modified();
      std::for_each(mpActors.begin(), mpActors.end(), [&] (auto& actor) { actor->GetProperty()->SetOpacity(val); actor->Modified(); });
    }

    /**
    */
    double VisualObject::getOpacity() const
    {
      return mpActor->GetProperty()->GetOpacity();
    }

    /**
    */
    bool VisualObject::getVisibility() const
    {
      return mpActor->GetVisibility() > 0;
    }
  }
}