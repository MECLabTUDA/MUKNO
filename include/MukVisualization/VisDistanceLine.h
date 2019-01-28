#pragma once

#include "VisAbstractObject.h"
#include "MukCommon/MukPath.h"


namespace gris
{
	namespace muk
	{
		/** \brief Visualization of the line that is used to measure distances between a waypoint and the closest risk structure.
		*/		
		class MUK_VIS_API VisDistanceLine : public VisAbstractObject
		{
		public:
			explicit VisDistanceLine(const std::string& name);
			virtual ~VisDistanceLine();

		public:
			void setLineWidth(double width);
			double getLineWidth() { return mLineWidth; }

			void setDistance(double distance);
			double getDistance() { return mDistance; }

			void setWarningDistance(double warningDistance);
			double getWarningDistance() { return mWarningDistance; }

			void                setWarningColor(const Vec3d& color);
			void                setWarningColor(double color[3]);
			const Vec3d&        getWarningColor() { return mWarningColor; }

		private:
			double mLineWidth;
			double mDistance;
			double mWarningDistance;
			Vec3d mWarningColor;

		};
	}
}
