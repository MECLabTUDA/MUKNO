#pragma once
#include "vtkCommand.h"

class vtkRenderer;

class CallbackSphereInteraction :
	public vtkCommand
{
public:

	static CallbackSphereInteraction* New();
	vtkTypeMacro(CallbackSphereInteraction, vtkCommand);
	~CallbackSphereInteraction()
	{
	}
	void setRenderer(vtkRenderer* pObj) { mpRenderer = pObj; };
	void func(vtkObject* implicitPlane, unsigned long eid, void* clientdata, void *calldata) {
	
	};

	vtkRenderer* mpRenderer;
};

