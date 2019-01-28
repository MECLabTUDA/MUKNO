#include "private/muk.pch"
#include "SphereRegionWidget.h"
#include "SphereRegionRepresentation.h"

#include "MukCommon/muk_common.h"

#include "vtkCamera.h"
#include "vtkCommand.h"
#include "vtkCallbackCommand.h"
#include "vtkEvent.h"
#include "vtkObjectFactory.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkStdString.h"
#include "vtkWidgetCallbackMapper.h"
#include "vtkWidgetEvent.h"
#include "vtkWidgetEventTranslator.h"

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
  vtkStandardNewMacro(SphereRegionWidget);

  //----------------------------------------------------------------------------
  SphereRegionWidget::SphereRegionWidget()
  {
    mWidgetState = enStart;
    ManagesCursor = 1;

    // Define widget events
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
      vtkWidgetEvent::Select,
      this, SphereRegionWidget::SelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
      vtkWidgetEvent::EndSelect,
      this, SphereRegionWidget::EndSelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent,
      vtkWidgetEvent::Move,
      this, SphereRegionWidget::MoveAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::KeyPressEvent,
      vtkEvent::AnyModifier, 30, 1, "Up",
      vtkWidgetEvent::Up,
      this, SphereRegionWidget::MoveAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::KeyPressEvent,
      vtkEvent::AnyModifier, 28, 1, "Right",
      vtkWidgetEvent::Up,
      this, SphereRegionWidget::MoveAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::KeyPressEvent,
      vtkEvent::AnyModifier, 31, 1, "Down",
      vtkWidgetEvent::Down,
      this, SphereRegionWidget::MoveAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::KeyPressEvent,
      vtkEvent::AnyModifier, 29, 1, "Left",
      vtkWidgetEvent::Down,
      this, SphereRegionWidget::MoveAction);
  }

  //----------------------------------------------------------------------------
  SphereRegionWidget::~SphereRegionWidget()
  {
  }

  //----------------------------------------------------------------------
  void SphereRegionWidget::SelectAction(vtkAbstractWidget *w)
  {
    auto* self = reinterpret_cast<SphereRegionWidget*>(w);
    auto* rep  = reinterpret_cast<SphereRegionRepresentation*>(self->WidgetRep);

    // Get the event position
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // make sure that the pick is in the current renderer
    if ( !self->CurrentRenderer || !self->CurrentRenderer->IsInViewport(X,Y) )
    {
      self->mWidgetState = enStart;
      return;
    }

    // update check for double click
    auto clickTime = std::chrono::high_resolution_clock::now();
    bool doubleClicked = std::chrono::duration_cast<std::chrono::milliseconds>(clickTime-S_Last_ClickTime).count() < 200;
    S_Last_ClickTime = clickTime;

    double eventPos[2];
    eventPos[0] = static_cast<double>(X);
    eventPos[1] = static_cast<double>(Y);
    rep->StartWidgetInteraction(eventPos);
    rep->ComputeInteractionState(X, Y);

    int interactionState = rep->GetInteractionState();
    interactionState = doubleClicked ? SphereRegionRepresentation::enPlaceSphere : interactionState;
    rep->SetInteractionState(interactionState);

    self->UpdateCursorShape(interactionState);
    if (interactionState == SphereRegionRepresentation::enOutside)
    {
      return;
    }

    // We are definitely selected
    self->mWidgetState = enActive;
    self->GrabFocus(self->EventCallbackCommand);

    if (doubleClicked)
    {
      rep->WidgetInteraction(eventPos);
    }
    
    self->EventCallbackCommand->SetAbortFlag(1);
    self->StartInteraction();
    self->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
    self->Render();
  }

  //----------------------------------------------------------------------
  void SphereRegionWidget::MoveAction(vtkAbstractWidget *w)
  {
    auto* self = reinterpret_cast<SphereRegionWidget*>(w);
    auto* rep  = reinterpret_cast<SphereRegionRepresentation*>(self->GetRepresentation());

    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];
    int changed = 0;

    // Whatever
    if (self->ManagesCursor && self->mWidgetState != SphereRegionWidget::enActive)
    {
      int oldInteractionState = rep->GetInteractionState();
      rep->SetInteractionState(SphereRegionRepresentation::enMovingMouse);
      int state = rep->ComputeInteractionState(X, Y);
      changed = self->UpdateCursorShape(state);
      rep->SetInteractionState(oldInteractionState);
      changed = (changed || state != oldInteractionState) ? 1 : 0;
    }

    // See whether we're active
    if (self->mWidgetState == SphereRegionWidget::enStart)
    {
      if (changed && self->ManagesCursor)
      {
        self->Render();
      }
      return;
    }

    //  adjust the representation
    double e[2];
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);
    rep->WidgetInteraction(e);

    // moving something
    self->EventCallbackCommand->SetAbortFlag(1);
    self->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
    self->Render();
  }

  //----------------------------------------------------------------------
  void SphereRegionWidget::EndSelectAction(vtkAbstractWidget *w)
  {
    auto* self = reinterpret_cast<SphereRegionWidget*>(w);
    auto* rep  = reinterpret_cast<SphereRegionRepresentation*>(self->GetRepresentation());
    if (self->mWidgetState != enActive  || rep->GetInteractionState() == SphereRegionRepresentation::enOutside)
    {
      return;
    }

    // Return state to not selected
    double e[2];
    rep->EndWidgetInteraction(e);
    self->mWidgetState = enStart;
    rep->SetInteractionState(SphereRegionRepresentation::enOutside);
    self->ReleaseFocus();

    // Update cursor if managed
    self->UpdateCursorShape(rep->GetInteractionState());

    self->EventCallbackCommand->SetAbortFlag(1);
    self->EndInteraction();
    self->InvokeEvent(vtkCommand::EndInteractionEvent, nullptr);
    self->Render();
  }

  //----------------------------------------------------------------------
  void SphereRegionWidget::SetProcessEvents(int enabling)
  {
    Superclass::SetProcessEvents(enabling);
    // the representation has its on interaction style that can e.g. enabled here
    //// switch visualization
    auto* rep = reinterpret_cast<SphereRegionRepresentation*>(WidgetRep);
    rep->setHandles(enabling==1);
  }

  //----------------------------------------------------------------------
  void SphereRegionWidget::CreateDefaultRepresentation()
  {
    if ( ! WidgetRep)
    {
      WidgetRep = SphereRegionRepresentation::New();
    }
  }
  
  //----------------------------------------------------------------------
  int SphereRegionWidget::UpdateCursorShape(int state)
  {
    // So as to change the cursor shape when the mouse is poised over
    // the widget.
    if (ManagesCursor)
    {
      if (state == SphereRegionRepresentation::enOutside)
      {
        return RequestCursorShape(VTK_CURSOR_DEFAULT);
      }
      else if (state == SphereRegionRepresentation::enRotating
        ||     state == SphereRegionRepresentation::enMovingSphere)
      {
        return RequestCursorShape(VTK_CURSOR_HAND);
      }
      else
      {
        return RequestCursorShape(VTK_CURSOR_DEFAULT);
      }
    }
    return 0;
  }

}
}