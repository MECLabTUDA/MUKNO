#pragma once
#include "private/IMukInteraction.h"

namespace gris
{
	namespace muk
	{
    class VisPlaneRegion;

		/** \brief Generates the interaction to define a Initial or Goal Region consisting of a plane and a direction hint
		*/
		class PlaneRegionInteraction : public IMukInteraction
		{
		  public:
			  static PlaneRegionInteraction* New();
			  vtkTypeMacro(PlaneRegionInteraction, vtkInteractorStyleTrackballCamera);

		  public:
			  PlaneRegionInteraction();

		  public:
        // VKT API
			  virtual void OnRightButtonUp();

        // Mukno API
			  virtual void initialize(VisPlaneRegion* pObj = nullptr);
        virtual void finalize();

      private:
        enum EnType
        {
          enAsStart,
          enAsGoal
        };

      private:
        void accept(EnType type);
			  void discard(); // return to DefaultInteraction3D
        void finishInteraction(); // return to DefaultInteraction3D

		  private:
        VisPlaneRegion* mpRegion = nullptr;
		};
	}
}
