#include "private/muk.pch"
#include "Cursor3DSynchronizer.h"
#include "SliceWidget.h"

#include "MukVisualization/Cursor3DWidget.h"
#include "MukVisualization/Cursor3DRepresentation.h"
#include "MukVisualization/ROIRepresentation.h"
#include "MukVisualization/ROIWidget.h"
#include "MukVisualization/SliceInteractorStyle.h"
#include "MukVisualization/SliceRenderGroup.h"

#include "MukCommon/muk_common.h"

#include <vtkCallbackCommand.h>
#include <vtkObjectFactory.h>
#include <vtkWidgetRepresentation.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>

#include <functional>

namespace gris
{
  namespace muk
  {
    vtkStandardNewMacro(Cursor3DSynchronizer);

    /**
    */
    Cursor3DSynchronizer::Cursor3DSynchronizer()
    {
    }

    /**
    */
    void Cursor3DSynchronizer::initialize(Cursor3DWidget* w1, Cursor3DWidget* w2, Cursor3DWidget* w3)
    {
      w1->RemoveObserver(this);
      w2->RemoveObserver(this);
      w3->RemoveObserver(this);
      mpCursorWidgets.clear();
      mpCursorWidgets.push_back(w1);
      mpCursorWidgets.push_back(w2);
      mpCursorWidgets.push_back(w3);
      w1->AddObserver(vtkCommand::InteractionEvent, this);
      w2->AddObserver(vtkCommand::InteractionEvent, this);
      w3->AddObserver(vtkCommand::InteractionEvent, this);
    }

    /**
    */
    void Cursor3DSynchronizer::initialize(SliceRenderGroup& g1_, SliceRenderGroup& g2_, SliceRenderGroup& g3_)
    {
      auto* g1 = &g1_;
      auto* g2 = &g2_;
      auto* g3 = &g3_;
      mpGroups.clear();
      mpGroups.push_back(g1);
      mpGroups.push_back(g2);
      mpGroups.push_back(g3);
      auto* w1 = dynamic_cast<SliceInteractorStyle*>(g1->getInteractor()->GetInteractorStyle());
      auto* w2 = dynamic_cast<SliceInteractorStyle*>(g2->getInteractor()->GetInteractorStyle());
      auto* w3 = dynamic_cast<SliceInteractorStyle*>(g3->getInteractor()->GetInteractorStyle());
      mpInteractors.clear();
      mpInteractors.push_back(w1);
      mpInteractors.push_back(w2);
      mpInteractors.push_back(w3);
      w1->AddObserver(SliceInteractorStyle::enSliceChangedEvent, this);
      w2->AddObserver(SliceInteractorStyle::enSliceChangedEvent, this);
      w3->AddObserver(SliceInteractorStyle::enSliceChangedEvent, this);
    }

    /**
    */
    void Cursor3DSynchronizer::initialize(ROIWidget* g1, ROIWidget* g2, ROIWidget* g3)
    {
      mpROIs.clear();
      if (g1)
        mpROIs.push_back(static_cast<ROIRepresentation*>(g1->GetRepresentation()));
      if (g2)
        mpROIs.push_back(static_cast<ROIRepresentation*>(g2->GetRepresentation()));
      if (g3)
        mpROIs.push_back(static_cast<ROIRepresentation*>(g3->GetRepresentation()));
    }

    /**
    */
    void Cursor3DSynchronizer::initialize(SliceWidget* g1, SliceWidget* g2, SliceWidget* g3)
    {
      mpSliceWidgets.clear();
      if (g1)
        mpSliceWidgets.push_back(g1);
      if (g2)
        mpSliceWidgets.push_back(g2);
      if (g3)
        mpSliceWidgets.push_back(g3);
    }

    /**
    */
    void Cursor3DSynchronizer::Execute(vtkObject* caller, unsigned long eventId, void*)
    {
      if (mpCursorWidgets.size() != 3)
        return;
      int idxIn(0), idxOut1(0), idxOut2(0);
      enum EnType
      {
        enNone,
        enWidget,
        enInteractor
      };
      EnType type = enNone;
      if (caller == mpCursorWidgets[0])
      {
        idxIn = 0;
        idxOut1 = 1;
        idxOut2 = 2;
        type = enWidget;
      }
      else if (caller == mpCursorWidgets[1])
      {
        idxIn = 1;
        idxOut1 = 0;
        idxOut2 = 2;
        type = enWidget;
      }
      else if (caller == mpCursorWidgets[2])
      {
        idxIn = 2;
        idxOut1 = 1;
        idxOut2 = 0;
        type = enWidget;
      }
      else if (caller == mpInteractors[0]
        ||     caller == mpInteractors[1]
        ||     caller == mpInteractors[2])
      {
        type = enInteractor;
      }

      switch(type)
      {
        case enNone:
        {
          break;
        }
        case enWidget:
        {
          auto rep = dynamic_cast<Cursor3DRepresentation*>(mpCursorWidgets[idxIn]->GetRepresentation());
          double d[3];
          rep->getPosition(d);

          for (int i(0); i < 3; ++i)
            mpGroups[i]->setCurrentSlice(d);

          mpCursorWidgets[idxOut1]->setPosition(d);
          mpCursorWidgets[idxOut1]->GetRepresentation()->BuildRepresentation();
          mpCursorWidgets[idxOut1]->GetInteractor()->Render();

          mpCursorWidgets[idxOut2]->setPosition(d);
          mpCursorWidgets[idxOut2]->GetRepresentation()->BuildRepresentation();
          mpCursorWidgets[idxOut2]->GetInteractor()->Render();

          for (int i(0); i < mpROIs.size(); ++i)
            mpROIs[i]->updateVisibility(d);
          for (int i(0); i < mpCursorWidgets.size(); ++i)
            mpCursorWidgets[i]->GetInteractor()->GetRenderWindow()->Render();
          for (int i(0); i < mpSliceWidgets.size(); ++i)
          {
            const auto info = mpGroups[i]->getCurrentSlice();
            mpSliceWidgets[i]->setSliceNumber(info.sliceNumber, info.maxSlices);
          }
          break;
        }
        case enInteractor:
        {
          double position[3];
          for (size_t i(0); i < 3; ++i)
          {
            auto info = mpGroups[i]->getCurrentSlice();
            position[info.orientation] = info.sliceCoord;
          }
          for (size_t i(0); i < 3; ++i)
          {
            mpCursorWidgets[i]->setPosition(position);
            mpCursorWidgets[i]->GetRepresentation()->BuildRepresentation();
            mpCursorWidgets[i]->GetInteractor()->Render();
          }
          for (int i(0); i < mpROIs.size(); ++i)
            mpROIs[i]->updateVisibility(position);
          for (int i(0); i < mpCursorWidgets.size(); ++i)
            mpCursorWidgets[i]->GetInteractor()->GetRenderWindow()->Render();
          for (int i(0); i < mpSliceWidgets.size(); ++i)
          {
            const auto info = mpGroups[i]->getCurrentSlice();
            mpSliceWidgets[i]->setSliceNumber(info.sliceNumber, info.maxSlices);
          }
          break;
        }
      }
    }
  }
}