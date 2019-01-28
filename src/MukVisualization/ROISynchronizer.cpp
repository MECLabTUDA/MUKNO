#include "private/muk.pch"
#include "ROISynchronizer.h"

#include "MukCommon/muk_common.h"

#include <vtkCallbackCommand.h>
#include <vtkObjectFactory.h>
#include <vtkWidgetRepresentation.h>
#include <vtkRenderWindowInteractor.h>

#include <functional>

namespace
{
  
}

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(ROISynchronizer);

  /**
  */
  ROISynchronizer::ROISynchronizer()
  {
    for (auto* obj : mpWidgets)
      obj->RemoveObserver(this);
  }

  /**
  */
  void ROISynchronizer::initialize(ROIWidget* w1, ROIWidget* w2, ROIWidget* w3)
  {
    w1->RemoveObserver(this);
    w2->RemoveObserver(this);
    w3->RemoveObserver(this);
    mpWidgets.clear();
    mpWidgets.push_back(w1);
    mpWidgets.push_back(w2);
    mpWidgets.push_back(w3);
    w1->AddObserver(vtkCommand::InteractionEvent, this);
    w2->AddObserver(vtkCommand::InteractionEvent, this);
    w3->AddObserver(vtkCommand::InteractionEvent, this);
  }


  /**
  */
  void ROISynchronizer::Execute(vtkObject* caller, unsigned long eventId, void*)
  {
    if (mpWidgets.size() != 3)
      return;
    int idxIn, idxOut1, idxOut2;
    if (caller == mpWidgets[0])
    {
      idxIn = 0;
      idxOut1 = 1;
      idxOut2 = 2;
    }
    else if (caller == mpWidgets[1])
    {
      idxIn = 1;
      idxOut1 = 0;
      idxOut2 = 2;
    }
    else if (caller == mpWidgets[2])
    {
      idxIn = 2;
      idxOut1 = 1;
      idxOut2 = 0;
    }
    auto* d = mpWidgets[idxIn]->GetRepresentation()->GetBounds();

    mpWidgets[idxOut1]->GetRepresentation()->PlaceWidget(d);
    mpWidgets[idxOut1]->GetRepresentation()->BuildRepresentation();
    mpWidgets[idxOut1]->GetInteractor()->Render();

    mpWidgets[idxOut2]->GetRepresentation()->PlaceWidget(d);
    mpWidgets[idxOut2]->GetRepresentation()->BuildRepresentation();
    mpWidgets[idxOut2]->GetInteractor()->Render();
  }
}
}