#include "private/muk.pch"
#include "Cursor3DWidget.h"
#include "Cursor3DRepresentation.h"

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
  vtkStandardNewMacro(Cursor3DWidget);

  //----------------------------------------------------------------------------
  Cursor3DWidget::Cursor3DWidget()
  {
    mWidgetState = Cursor3DWidget::enStart;
    ManagesCursor = 1;

    // Define widget events
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
      vtkEvent::NoModifier,
      0, 0, nullptr,
      vtkWidgetEvent::Select,
      this, Cursor3DWidget::SelectAction);
    CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
      vtkEvent::NoModifier,
      0, 0, nullptr,
      vtkWidgetEvent::EndSelect,
      this, Cursor3DWidget::EndSelectAction);

    CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent,
      vtkWidgetEvent::Move,
      this, Cursor3DWidget::MoveAction);
  }

  //----------------------------------------------------------------------------
  Cursor3DWidget::~Cursor3DWidget()
  {
  }

  //----------------------------------------------------------------------
  void Cursor3DWidget::SelectAction(vtkAbstractWidget *w)
  {
    // We are in a static method, cast to ourself
    Cursor3DWidget *self = reinterpret_cast<Cursor3DWidget*>(w);

    // Get the event position
    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // Okay, make sure that the pick is in the current renderer
    if ( ! self->CurrentRenderer || ! self->CurrentRenderer->IsInViewport(X,Y) )
    {
      self->mWidgetState = Cursor3DWidget::enStart;
      return;
    }

    // Begin the widget interaction which has the side effect of setting the
    // interaction state.
    double e[2];
    e[0] = static_cast<double>(X);
    e[1] = static_cast<double>(Y);
    self->WidgetRep->StartWidgetInteraction(e);
    int interactionState = self->WidgetRep->GetInteractionState();
    if ( interactionState == Cursor3DRepresentation::enOutside )
    {
      return;
    }

    // We are definitely selected
    self->mWidgetState = Cursor3DWidget::enActive;
    self->GrabFocus(self->EventCallbackCommand);

    // The SetInteractionState has the side effect of highlighting the widget
    reinterpret_cast<Cursor3DRepresentation*>(self->WidgetRep)->
      SetInteractionState(interactionState);

    // start the interaction
    self->EventCallbackCommand->SetAbortFlag(1);
    self->StartInteraction();
    self->InvokeEvent(vtkCommand::StartInteractionEvent,nullptr);
    self->Render();

    // don't know how to prioritize widgets in CTor
    // immediately start interacting
    MoveAction(w);
  }
  
  //----------------------------------------------------------------------
  void Cursor3DWidget::MoveAction(vtkAbstractWidget *w)
  {
    Cursor3DWidget *self = reinterpret_cast<Cursor3DWidget*>(w);

    // See whether we're active
    if ( self->mWidgetState == Cursor3DWidget::enStart )
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
  void Cursor3DWidget::EndSelectAction(vtkAbstractWidget *w)
  {
    Cursor3DWidget *self = reinterpret_cast<Cursor3DWidget*>(w);
    if ( self->mWidgetState == Cursor3DWidget::enStart )
    {
      return;
    }

    // Return state to not active
    self->mWidgetState = Cursor3DWidget::enStart;
    reinterpret_cast<Cursor3DRepresentation*>(self->WidgetRep)->
      SetInteractionState(Cursor3DRepresentation::enOutside);
    self->ReleaseFocus();

    self->EventCallbackCommand->SetAbortFlag(1);
    self->EndInteraction();
    self->InvokeEvent(vtkCommand::EndInteractionEvent,nullptr);
    self->Render();
  }

  //----------------------------------------------------------------------
  void Cursor3DWidget::CreateDefaultRepresentation()
  {
    if ( ! WidgetRep )
    {
      WidgetRep = Cursor3DRepresentation::New();
    }
  }

  //----------------------------------------------------------------------
  /**
  */
  void Cursor3DWidget::setPosition(double pos[3])
  {
    reinterpret_cast<Cursor3DRepresentation*>(WidgetRep)->setPosition(pos);
  }

  //----------------------------------------------------------------------
  /**
  */
  void Cursor3DWidget::setImage(vtkImageData* pImage)
  {
    reinterpret_cast<Cursor3DRepresentation*>(WidgetRep)->setImage(pImage);
  }
}
}