#include "private/muk.pch"
#include "ROIWidget.h"
#include "ROIRepresentation.h"

#include "vtkCommand.h"
#include "vtkCallbackCommand.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkObjectFactory.h"
#include "vtkWidgetEventTranslator.h"
#include "vtkWidgetCallbackMapper.h"
#include "vtkEvent.h"
#include "vtkWidgetEvent.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"

namespace gris
{
namespace muk
{

vtkStandardNewMacro(ROIWidget);

//----------------------------------------------------------------------------
ROIWidget::ROIWidget()
{
  this->mWidgetState = ROIWidget::enStart;
  this->ManagesCursor = 1;
  
  // Define widget events
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
    vtkEvent::NoModifier,
    0, 0, nullptr,
    vtkWidgetEvent::Select,
    this, ROIWidget::SelectAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
    vtkEvent::NoModifier,
    0, 0, nullptr,
    vtkWidgetEvent::EndSelect,
    this, ROIWidget::EndSelectAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::MiddleButtonPressEvent,
    vtkWidgetEvent::Translate,
    this, ROIWidget::TranslateAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::MiddleButtonReleaseEvent,
    vtkWidgetEvent::EndTranslate,
    this, ROIWidget::EndSelectAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
    vtkEvent::ControlModifier,
    0, 0, nullptr,
    vtkWidgetEvent::Translate,
    this, ROIWidget::TranslateAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
    vtkEvent::ControlModifier,
    0, 0, nullptr,
    vtkWidgetEvent::EndTranslate,
    this, ROIWidget::EndSelectAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
    vtkEvent::ShiftModifier,
    0, 0, nullptr,
    vtkWidgetEvent::Translate,
    this, ROIWidget::TranslateAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
    vtkEvent::ShiftModifier,
    0, 0, nullptr,
    vtkWidgetEvent::EndTranslate,
    this, ROIWidget::EndSelectAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::RightButtonPressEvent,
    vtkWidgetEvent::Scale,
    this, ROIWidget::ScaleAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::RightButtonReleaseEvent,
    vtkWidgetEvent::EndScale,
    this, ROIWidget::EndSelectAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent,
    vtkWidgetEvent::Move,
    this, ROIWidget::MoveAction);
}

//----------------------------------------------------------------------------
ROIWidget::~ROIWidget()
{
}

//----------------------------------------------------------------------
void ROIWidget::SelectAction(vtkAbstractWidget *w)
{
  // We are in a static method, cast to ourself
  ROIWidget *self = reinterpret_cast<ROIWidget*>(w);

  // Get the event position
  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];

  // Okay, make sure that the pick is in the current renderer
  if ( !self->CurrentRenderer ||
    !self->CurrentRenderer->IsInViewport(X,Y) )
  {
    self->mWidgetState = ROIWidget::enStart;
    return;
  }

  // Begin the widget interaction which has the side effect of setting the
  // interaction state.
  double e[2];
  e[0] = static_cast<double>(X);
  e[1] = static_cast<double>(Y);
  self->WidgetRep->StartWidgetInteraction(e);
  int interactionState = self->WidgetRep->GetInteractionState();
  if ( interactionState == ROIRepresentation::Outside )
  {
    return;
  }

  // Rotation
  if (interactionState == ROIRepresentation::Rotating)
  {
    return;
  }
  
  // We are definitely selected
  self->mWidgetState = ROIWidget::enActive;
  self->GrabFocus(self->EventCallbackCommand);

  // The SetInteractionState has the side effect of highlighting the widget
  reinterpret_cast<ROIRepresentation*>(self->WidgetRep)->
    SetInteractionState(interactionState);

  // start the interaction
  self->EventCallbackCommand->SetAbortFlag(1);
  self->StartInteraction();
  self->InvokeEvent(vtkCommand::StartInteractionEvent,nullptr);
  self->Render();
}

//----------------------------------------------------------------------
void ROIWidget::TranslateAction(vtkAbstractWidget *w)
{
  // We are in a static method, cast to ourself
  ROIWidget *self = reinterpret_cast<ROIWidget*>(w);

  // Get the event position
  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];

  // Okay, make sure that the pick is in the current renderer
  if ( !self->CurrentRenderer ||
    !self->CurrentRenderer->IsInViewport(X,Y) )
  {
    self->mWidgetState = ROIWidget::enStart;
    return;
  }

  // Begin the widget interaction which has the side effect of setting the
  // interaction state.
  double e[2];
  e[0] = static_cast<double>(X);
  e[1] = static_cast<double>(Y);
  self->WidgetRep->StartWidgetInteraction(e);
  int interactionState = self->WidgetRep->GetInteractionState();
  if ( interactionState == ROIRepresentation::Outside )
  {
    return;
  }

  // We are definitely selected
  self->mWidgetState = ROIWidget::enActive;
  self->GrabFocus(self->EventCallbackCommand);
  reinterpret_cast<ROIRepresentation*>(self->WidgetRep)->
    SetInteractionState(ROIRepresentation::Translating);

  // start the interaction
  self->EventCallbackCommand->SetAbortFlag(1);
  self->StartInteraction();
  self->InvokeEvent(vtkCommand::StartInteractionEvent,nullptr);
  self->Render();
}

//----------------------------------------------------------------------
void ROIWidget::ScaleAction(vtkAbstractWidget *w)
{
  // We are in a static method, cast to ourself
  ROIWidget *self = reinterpret_cast<ROIWidget*>(w);

  // Get the event position
  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];

  // Okay, make sure that the pick is in the current renderer
  if ( !self->CurrentRenderer ||
    !self->CurrentRenderer->IsInViewport(X,Y) )
  {
    self->mWidgetState = ROIWidget::enStart;
    return;
  }

  // Begin the widget interaction which has the side effect of setting the
  // interaction state.
  double e[2];
  e[0] = static_cast<double>(X);
  e[1] = static_cast<double>(Y);
  self->WidgetRep->StartWidgetInteraction(e);
  int interactionState = self->WidgetRep->GetInteractionState();
  if ( interactionState == ROIRepresentation::Outside )
  {
    return;
  }

  // We are definitely selected
  self->mWidgetState = ROIWidget::enActive;
  self->GrabFocus(self->EventCallbackCommand);
  reinterpret_cast<ROIRepresentation*>(self->WidgetRep)->
    SetInteractionState(ROIRepresentation::Scaling);

  // start the interaction
  self->EventCallbackCommand->SetAbortFlag(1);
  self->StartInteraction();
  self->InvokeEvent(vtkCommand::StartInteractionEvent,nullptr);
  self->Render();
}

//----------------------------------------------------------------------
void ROIWidget::MoveAction(vtkAbstractWidget *w)
{
  ROIWidget *self = reinterpret_cast<ROIWidget*>(w);

  // See whether we're active
  if ( self->mWidgetState == ROIWidget::enStart )
  {
    return;
  }

  // compute some info we need for all cases
  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];

  // Okay, adjust the representation
  double e[2];
  e[0] = static_cast<double>(X);
  e[1] = static_cast<double>(Y);
  self->WidgetRep->WidgetInteraction(e);

  // moving something
  self->EventCallbackCommand->SetAbortFlag(1);
  self->InvokeEvent(vtkCommand::InteractionEvent,nullptr);
  self->Render();
}

//----------------------------------------------------------------------
void ROIWidget::EndSelectAction(vtkAbstractWidget *w)
{
  ROIWidget *self = reinterpret_cast<ROIWidget*>(w);
  if ( self->mWidgetState == ROIWidget::enStart )
  {
    return;
  }

  // Return state to not active
  self->mWidgetState = ROIWidget::enStart;
  reinterpret_cast<ROIRepresentation*>(self->WidgetRep)->
    SetInteractionState(ROIRepresentation::Outside);
  self->ReleaseFocus();

  self->EventCallbackCommand->SetAbortFlag(1);
  self->EndInteraction();
  self->InvokeEvent(vtkCommand::EndInteractionEvent,nullptr);
  self->Render();
}

//----------------------------------------------------------------------
void ROIWidget::CreateDefaultRepresentation()
{
  if ( ! this->WidgetRep )
  {
    this->WidgetRep = ROIRepresentation::New();
  }
}

//----------------------------------------------------------------------------
void ROIWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

}
}