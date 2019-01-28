#include "private/muk.pch"
#include "VisAxes.h"

#include <vtkArrowSource.h>
#include <vtkAppendPolyData.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include "vtkActor.h"
#include "vtkProperty.h"

namespace gris
{
	namespace muk
	{
		/**
		*/
		VisAxes::VisAxes(const std::string& name) 
      : VisAbstractObject(name)
		{
      setDefaultColor(Vec3d(0.82, 0.82, 0.82));
      setLineStipplePattern(0xaaaa);
		}

		/**
		*/
		VisAxes::~VisAxes()
		{
		}

		/** \brief Sets the line stipple pattern of the lines as a 16-bit binary pattern (1 = pixel on, 0 = pixel off). 
			
			See: vtkProperty2D::SetLineStipplePattern
		*/		
		void VisAxes::setLineStipplePattern(int pattern)
		{
			mpActor->GetProperty()->SetLineStipplePattern(pattern);
			std::for_each(mpActors.begin(), mpActors.end(), [&](auto& actor) { actor->GetProperty()->SetLineStipplePattern(pattern); });
		}




	}
}