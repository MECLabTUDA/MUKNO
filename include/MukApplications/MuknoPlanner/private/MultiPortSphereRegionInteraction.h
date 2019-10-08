#pragma once
#include "private/IMukInteraction.h"
#include "private/ImplicitPlaneRepresentation.h"
#include "VisAbstractObject.h"
#include "MukCommon/MukVector.h"

namespace gris
{
	namespace muk
	{
		/** \brief Generates the interaction to define a Initial or Goal Region consisting of a ball and up to 3 directions
		*/
		class MultiPortSphereRegionInteraction : public IMukInteraction
		{
		  public:
			  static MultiPortSphereRegionInteraction* New();
			  vtkTypeMacro(MultiPortSphereRegionInteraction, IMukInteraction);

		  public:
        MultiPortSphereRegionInteraction();

		  public:
        // VTK API
			  virtual void OnRightButtonUp();

        // Mukno API
			  virtual void initialize();
        virtual void finalize() {}

    public:
        void setSphereRadius(double r);
        void setPathRadius(double r);
        void setAngleThreshold(double alpha);
			  void discard(); // return to DefaultInteraction3D
        void acceptAsStart();
        void acceptAsGoal();

		  private:
			  void finishInteraction(); // return to DefaultInteraction3D

      private:
        double mPathRadius   = 1.0;
        double mSphereRadius = 1.0;
        double mAngleThreshold = 0.1;
		};
	}
}
