#include "private/muk.pch"
#include "SurfaceRegionWidget.h"
#include "SurfaceRegionRepresentation.h"

#include "MukCommon/muk_common.h"

#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkEvent.h>
#include <vtkObjectFactory.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkStdString.h>
#include <vtkWidgetCallbackMapper.h>
#include <vtkWidgetEvent.h>
#include <vtkWidgetEventTranslator.h>

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
  vtkStandardNewMacro(SurfaceRegionWidget);


  //----------------------------------------------------------------------------
  SurfaceRegionWidget::SurfaceRegionWidget()
  {
    mWidgetState = SurfaceRegionWidget::Start;
    //ManagesCursor = 1;

    // Define widget events
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
      vtkEvent::NoModifier,
      0, 0, nullptr,
      vtkWidgetEvent::Select,
      this, SurfaceRegionWidget::SelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
      vtkEvent::NoModifier,
      0, 0, nullptr,
      vtkWidgetEvent::EndSelect,
      this, SurfaceRegionWidget::EndSelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::MiddleButtonPressEvent,
      vtkWidgetEvent::Translate,
      this, SurfaceRegionWidget::ScrollAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::MiddleButtonReleaseEvent,
      vtkWidgetEvent::EndTranslate,
      this, SurfaceRegionWidget::EndSelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
      vtkEvent::ControlModifier,
      0, 0, nullptr,
      vtkWidgetEvent::Translate,
      this, SurfaceRegionWidget::ScrollAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
      vtkEvent::ControlModifier,
      0, 0, nullptr,
      vtkWidgetEvent::EndTranslate,
      this, SurfaceRegionWidget::EndSelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
      vtkEvent::ShiftModifier,
      0, 0, nullptr,
      vtkWidgetEvent::Translate,
      this, SurfaceRegionWidget::ScrollAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
      vtkEvent::ShiftModifier,
      0, 0, nullptr,
      vtkWidgetEvent::EndTranslate,
      this, SurfaceRegionWidget::EndSelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::RightButtonPressEvent,
      vtkWidgetEvent::Scale,
      this, SurfaceRegionWidget::ScrollAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::RightButtonReleaseEvent,
      vtkWidgetEvent::EndScale,
      this, SurfaceRegionWidget::EndSelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent,
      vtkWidgetEvent::Move,
      this, SurfaceRegionWidget::MoveAction);
  }

  //----------------------------------------------------------------------------
  SurfaceRegionWidget::~SurfaceRegionWidget()
  {
  }

  //----------------------------------------------------------------------
  void SurfaceRegionWidget::SelectAction(vtkAbstractWidget *w)
  {
    // We are in a static method, cast to ourself
    auto* self = reinterpret_cast<SurfaceRegionWidget*>(w);
    auto* rep  = reinterpret_cast<SurfaceRegionRepresentation*>(self->WidgetRep);

    // Get the event position
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // make sure that the pick is in the current renderer
    if ( !self->CurrentRenderer || !self->CurrentRenderer->IsInViewport(X,Y) )
    {
      self->mWidgetState = Start;
      return;
    }

    // update check for double click
    auto clickTime = std::chrono::high_resolution_clock::now();
    bool doubleClicked = std::chrono::duration_cast<std::chrono::milliseconds>(clickTime-S_Last_ClickTime).count() < 200;
    S_Last_ClickTime = clickTime;

    // Begin the widget interaction which has the side effect of setting the
    // interaction state.
    double e[2];
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);
    rep->StartWidgetInteraction(e);
    int interactionState = doubleClicked ? SurfaceRegionRepresentation::enPlaceSphere : rep->GetInteractionState();
    if (interactionState == SurfaceRegionRepresentation::enOutside)
    {
      return;
    }

    // Test for states that involve face or handle picking here so
    // selection highlighting doesn't happen if that interaction is disabled.
    // Non-handle-grabbing transformations are tested in the "Action" methods.

    // We are definitely selected
    self->mWidgetState = Active;
    self->GrabFocus(self->EventCallbackCommand);

    // The SetInteractionState has the side effect of highlighting the widget
    rep->SetInteractionState(interactionState);

    if (doubleClicked)
    {
      rep->WidgetInteraction(e);
    }

    // start the interaction
    self->EventCallbackCommand->SetAbortFlag(1);
    self->StartInteraction();
    self->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
    self->Render();
  }

  //----------------------------------------------------------------------
  void SurfaceRegionWidget::ScrollAction(vtkAbstractWidget *w)
  {
    // We are in a static method, cast to ourself
    SurfaceRegionWidget *self = reinterpret_cast<SurfaceRegionWidget*>(w);

    // Get the event position
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // Okay, make sure that the pick is in the current renderer
    if ( !self->CurrentRenderer || !self->CurrentRenderer->IsInViewport(X,Y) )
    {
      self->mWidgetState = SurfaceRegionWidget::Start;
      return;
    }

    // Begin the widget interaction which has the side effect of setting the
    // interaction state.
    double e[2];
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);
    self->WidgetRep->StartWidgetInteraction(e);
    int interactionState = self->WidgetRep->GetInteractionState();
    if ( interactionState == SurfaceRegionRepresentation::enOutside )
    {
      return;
    }

    // We are definitely selected
    self->mWidgetState = SurfaceRegionWidget::Active;
    self->GrabFocus(self->EventCallbackCommand);
    reinterpret_cast<SurfaceRegionRepresentation*>(self->WidgetRep)->SetInteractionState(SurfaceRegionRepresentation::enScrollSphere);

    // start the interaction
    self->EventCallbackCommand->SetAbortFlag(1);
    self->StartInteraction();
    self->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
    self->Render();
  }

  //----------------------------------------------------------------------
  void SurfaceRegionWidget::MoveAction(vtkAbstractWidget *w)
  {
    SurfaceRegionWidget *self = reinterpret_cast<SurfaceRegionWidget*>(w);

    // See whether we're active
    if ( self->mWidgetState == SurfaceRegionWidget::Start )
    {
      return;
    }

    // Okay, adjust the representation
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];
    double e[2];
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);
    self->WidgetRep->WidgetInteraction(e);

    // moving something
    self->EventCallbackCommand->SetAbortFlag(1);
    self->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
    self->Render();
  }

  //----------------------------------------------------------------------
  void SurfaceRegionWidget::EndSelectAction(vtkAbstractWidget *w)
  {
    SurfaceRegionWidget *self = reinterpret_cast<SurfaceRegionWidget*>(w);
    if ( self->mWidgetState == SurfaceRegionWidget::Start )
    {
      return;
    }

    // Return state to not active
    self->mWidgetState = SurfaceRegionWidget::Start;
    reinterpret_cast<SurfaceRegionRepresentation*>(self->WidgetRep)->
      SetInteractionState(SurfaceRegionRepresentation::enOutside);
    self->ReleaseFocus();

    self->EventCallbackCommand->SetAbortFlag(1);
    self->EndInteraction();
    self->InvokeEvent(vtkCommand::EndInteractionEvent, nullptr);
    self->Render();
  }

  //----------------------------------------------------------------------
  void SurfaceRegionWidget::CreateDefaultRepresentation()
  {
    if ( ! WidgetRep )
    {
      WidgetRep = SurfaceRegionRepresentation::New();
    }
  }

  //----------------------------------------------------------------------------
  /**
  */
  void SurfaceRegionWidget::PrintSelf(ostream& os, vtkIndent indent)
  {
    Superclass::PrintSelf(os,indent);
  }

  //----------------------------------------------------------------------------
  /**
  */
  void SurfaceRegionWidget::SetEnabled(int on)
  {
    Superclass::SetEnabled(on);
  }

  //----------------------------------------------------------------------------
  /**
  */
  void SurfaceRegionWidget::SetProcessEvents(int on)
  {
    Superclass::SetProcessEvents(on);
    // the representation has its on interaction style that can e.g. enabled here
    auto* rep = reinterpret_cast<SurfaceRegionRepresentation*>(WidgetRep);
    rep->setHandles(on==1);
  }
}
}