#pragma once

#include "VisAbstractObject.h"
#include "MukCommon/MukPath.h"
#include "vtkTexture.h"

namespace gris
{
	namespace muk
	{
		/** \brief Visualization of plane that supports a custom texture
		*/		
		class MUK_VIS_API Vis3dPlane : public VisAbstractObject
		{
		public:
			explicit Vis3dPlane(const std::string& name);
			virtual ~Vis3dPlane();

		public:
			void setTexture(vtkTexture* texture);

		private:
			
		};
	}
}
