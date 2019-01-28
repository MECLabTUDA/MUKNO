#pragma once

#include "VisAbstractObject.h"
#include "MukCommon/MukPath.h"
#include "vtkTexture.h"

namespace gris
{
	namespace muk
	{
		/** \brief Visualization of coordinate axes that supports a custom line stippling pattern
		*/
		class MUK_VIS_API VisAxes : public VisAbstractObject
		{
		public:
			explicit VisAxes(const std::string& name);
      virtual ~VisAxes();

		public:
			void setLineStipplePattern(int pattern);

		private:

		};
	}
}
