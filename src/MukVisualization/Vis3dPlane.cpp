#include "private/muk.pch"
#include "Vis3dPlane.h"

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
		Vis3dPlane::Vis3dPlane(const std::string& name) 
      : VisAbstractObject(name)
		{
      setDefaultColor(Vec3d(0.82, 0.82, 0.82));
      setOpacity(0.5);
		}

		/**
		*/
		Vis3dPlane::~Vis3dPlane()
		{
		}

		/** \brief Sets the texture the plane should use
		*/
		void Vis3dPlane::setTexture(vtkTexture* texture)
		{
			mpActor->SetTexture(texture);
			std::for_each(mpActors.begin(), mpActors.end(), [&](auto& actor) { actor->SetTexture(texture); });
		}

	


	}
}