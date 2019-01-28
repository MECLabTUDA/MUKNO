#include "private/muk.pch"
#include "PlaneRegionWidget.h"
#include "PlaneRegionRepresentation.h"

#include <vtkCallbackCommand.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkWidgetCallbackMapper.h>
#include "vtkWidgetEvent.h"

#include <chrono>

namespace
{
  using namespace std::chrono;
  static time_point<steady_clock> S_Last_ClickTime;
}

namespace gris
{
namespace muk
{
  vtkStandardNewMacro(PlaneRegionWidget);

  /**
  */
  PlaneRegionWidget::PlaneRegionWidget()
  {
    // overwrite baseclass behavior
    this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
      vtkWidgetEvent::Select,
      this, PlaneRegionWidget::SelectAction);
    this->CallbackMapper->SetCallbackMethod(vtkCommand::RightButtonPressEvent,
      vtkWidgetEvent::Scale,
      this, PlaneRegionWidget::ScaleAction);
  }
  
  /**
  */
  PlaneRegionWidget::~PlaneRegionWidget()
  {
  }
  
  /**
  */
  void PlaneRegionWidget::CreateDefaultRepresentation()
  {
    if ( ! WidgetRep )
    {
      WidgetRep = PlaneRegionRepresentation::New();
    }
  }

  /**
  */
  void PlaneRegionWidget::SetProcessEvents(int on)
  {
    Superclass::SetProcessEvents(on);
    auto* rep = reinterpret_cast<PlaneRegionRepresentation*>(WidgetRep);
    rep->setHandles(on==1);
  }

  /**
  */
  void PlaneRegionWidget::SelectAction(vtkAbstractWidget* w)
  {
    auto* self = reinterpret_cast<PlaneRegionWidget*>(w);
    auto* rep  = reinterpret_cast<PlaneRegionRepresentation*>(w->GetRepresentation());

    // Get the event position
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // make sure that the pick is in the current renderer
    if ( !self->CurrentRenderer || !self->CurrentRenderer->IsInViewport(X,Y) )
    {
      self->WidgetState = Start;
      return;
    }

    // update check for double click
    auto clickTime = std::chrono::high_resolution_clock::now();
    const bool doubleClicked = std::chrono::duration_cast<std::chrono::milliseconds>(clickTime-S_Last_ClickTime).count() < 200;
    S_Last_ClickTime = clickTime;
    
    if (doubleClicked)
    {
      int interactionState;
      if (self->Interactor->GetControlKey())
        interactionState = PlaneRegionRepresentation::enPlaceAncor;
      else
        interactionState = PlaneRegionRepresentation::enPlacePlane;
      rep->SetInteractionState(interactionState);
    }
    else
    {
      // We want to compute an orthogonal vector to the plane that has been selected
      rep->SetInteractionState(PlaneRegionRepresentation::enMoving);
      auto interactionState = self->WidgetRep->ComputeInteractionState(X, Y);
      self->UpdateCursorShape(interactionState);
    }
    if ( rep->GetInteractionState() == PlaneRegionRepresentation::enOutside )
    {
      return;
    }

    // We are definitely selected
    double eventPos[2];
    eventPos[0] = static_cast<double>(X);
    eventPos[1] = static_cast<double>(Y);
    self->GrabFocus(self->EventCallbackCommand);
    if (doubleClicked)
    {
      rep->WidgetInteraction(eventPos);
    }

    self->WidgetState = Active;
    rep->StartWidgetInteraction(eventPos);

    self->EventCallbackCommand->SetAbortFlag(1);
    self->StartInteraction();
    self->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
    self->Render(); 
  }

  /**
  */
  void PlaneRegionWidget::ScaleAction(vtkAbstractWidget* w)
  {
    auto* self = reinterpret_cast<PlaneRegionWidget*>(w);
    auto* rep  = reinterpret_cast<PlaneRegionRepresentation*>(self->GetRepresentation());

    // Get the event position
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // update check for double click
    auto clickTime = std::chrono::high_resolution_clock::now();
    // this can be a double right click (or better a left followed by right click)
    const bool doubleClicked = std::chrono::duration_cast<std::chrono::milliseconds>(clickTime-S_Last_ClickTime).count() < 500;
    S_Last_ClickTime = clickTime;
    
    if (doubleClicked)
    {
      rep->SetInteractionState(PlaneRegionRepresentation::enPlaceAncor);
    }
    else
    {
      // We want to compute an orthogonal vector to the pane that has been selected
      rep->SetInteractionState(PlaneRegionRepresentation::enScaling);
      int interactionState = rep->ComputeInteractionState(X, Y);
      self->UpdateCursorShape(interactionState);
    }
    if ( rep->GetInteractionState() == PlaneRegionRepresentation::enOutside )
    {
      return;
    }

    // We are definitely selected
    double eventPos[2];
    eventPos[0] = static_cast<double>(X);
    eventPos[1] = static_cast<double>(Y);
    self->GrabFocus(self->EventCallbackCommand);
    if (doubleClicked)
    {
      rep->WidgetInteraction(eventPos);
    }

    self->WidgetState = Active;
    rep->StartWidgetInteraction(eventPos);

    self->EventCallbackCommand->SetAbortFlag(1);
    self->StartInteraction();
    self->InvokeEvent(vtkCommand::StartInteractionEvent,nullptr);
    self->Render();
  }
}
}