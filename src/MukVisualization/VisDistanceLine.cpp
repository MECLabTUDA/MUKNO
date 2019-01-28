#include "private/muk.pch"
#include "VisDistanceLine.h"

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

#include "MukCommon/overload_deduction.h"

namespace gris
{
	namespace muk
	{
		/** \brief Default Constructor
		*/
		VisDistanceLine::VisDistanceLine(const std::string& name) 
      : VisAbstractObject(name)
      , mLineWidth(1.0)
      , mWarningColor(Vec3d(1.0, 1.0, 0.0))
      , mWarningDistance(1.0)
		{
			declareProperty<double>("lineWidth",
				std::bind(&VisDistanceLine::setLineWidth, this, std::placeholders::_1),
				std::bind(&VisDistanceLine::getLineWidth, this));
			
			declareProperty<double>("warningDistance",
				std::bind(&VisDistanceLine::setWarningDistance, this, std::placeholders::_1),
				std::bind(&VisDistanceLine::getWarningDistance, this));

			declareProperty<Vec3d>("warningColor",
				std::bind(SELECT<const Vec3d&>::OVERLOAD_OF(&VisDistanceLine::setWarningColor), this, std::placeholders::_1),
				std::bind(&VisDistanceLine::getWarningColor, this));

			setLineWidth(mLineWidth);
      setDefaultColor(Vec3d(1, 0, 0.5));
		}

		/**
		*/
		VisDistanceLine::~VisDistanceLine()
		{
		}

		/** 
		*/
		void VisDistanceLine::setLineWidth(double width)
		{
			mLineWidth = width;
			mpActor->GetProperty()->SetLineWidth(mLineWidth);
			std::for_each(mpActors.begin(), mpActors.end(), [&](auto& actor) { actor->GetProperty()->SetLineWidth(mLineWidth); });
		}

		/** \brief Sets the @distance and changes the color of  the line, if @distance is smaller than mWarningDistance
		*/		
		void VisDistanceLine::setDistance(double distance)
		{
			mDistance = distance;
			if (distance < mWarningDistance)
			{
				mpActor->GetProperty()->SetColor(mWarningColor.data());
				std::for_each(mpActors.begin(), mpActors.end(), [&](auto& actor) { actor->GetProperty()->SetColor(mWarningColor.data()); } );
			}
			else
			{
				mpActor->GetProperty()->SetColor(mDefaultColor.data());
				std::for_each(mpActors.begin(), mpActors.end(), [&](auto& actor) { actor->GetProperty()->SetColor(mDefaultColor.data()); } );
			}
		}

		/** \brief Sets the @warningDistance which dictates when a change in color of the line is necessary
		*/		
		void VisDistanceLine::setWarningDistance(double warningDistance)
		{
			mWarningDistance = warningDistance;
		}

		/**
		*/
		void VisDistanceLine::setWarningColor(const Vec3d& color)
		{
			mWarningColor = color;
		}

		/** \brief
		*/
		void VisDistanceLine::setWarningColor(double color[3])
		{
			setWarningColor(Vec3d(color[0], color[1], color[2]));
		}
	}
}