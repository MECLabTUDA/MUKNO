#pragma once
#include "vtkInteractorStyleTrackballCamera.h"
namespace gris
{
	namespace muk
	{
		class CostumInteractorStyle :
			public vtkInteractorStyleTrackballCamera
		{
		public:
			CostumInteractorStyle();
			~CostumInteractorStyle();
		};
	};
}
