#include "private/muk.pch"
#include "MeshReduction.h"

#include "AlgorithmFactory.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkDecimatePro.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkPoints.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(MeshReduction);

  /**
  */
  MeshReduction::MeshReduction()
    : mNumberOfPoints(1000)
    , mMinimumPoints(100)
  {
    mDspType = enDisplayPolyData;
    mInputPortTypes.push_back(enVtkPolyData);
    mOutputPortTypes.push_back(enVtkPolyData);
    // atm: allow only one countour

    mpFilter = make_vtk<vtkDecimatePro>();
    mpOutput  = make_vtk<vtkPolyData>();
    
    auto* ptr = static_cast<vtkDecimatePro*>(mpFilter.GetPointer());
    ptr->SetSplitting(false);
    ptr->SetPreserveTopology(false);
    ptr->SetBoundaryVertexDeletion(false);
    ptr->SetTargetReduction(0.5);
    ptr->SetMaximumError(VTK_DOUBLE_MAX);

    declareProperty<double>("TargetReduction",
      [=] (double v) { ptr->SetTargetReduction(v); },
      [=] ()         { return ptr->GetTargetReduction();  });
    declareProperty<int>("PreserveTopology",
      [=] (int v) { ptr->SetPreserveTopology(v); },
      [=] ()      { return ptr->GetPreserveTopology();  });
    declareProperty<int>("EnableSplitting",
      [=] (int v) { ptr->SetSplitting(v); },
      [=] ()      { return ptr->GetSplitting();  });
    declareProperty<int>("EnableBoundaryVertexDeletion",
      [=] (int v) { ptr->SetBoundaryVertexDeletion(v); },
      [=] ()      { return ptr->GetBoundaryVertexDeletion();  });
    declareProperty<size_t >("NumberOfPoints",
      [=] (size_t v) { setNumberofPoints(v); },
      [=] ()         { return getNumberofPoints(); });
  }

  /**
  */
  void MeshReduction::setInput(unsigned int portId, void* pDataType)
  {
    mpInput = static_cast<vtkPolyData*>(pDataType);
  }

  /**
  */
  void* MeshReduction::getOutput(unsigned int portId)
  {
    return static_cast<void*>(mpOutput.GetPointer());
  }

  /**
  */
  void MeshReduction::setNumberofPoints(size_t N)
  {
    mNumberOfPoints = N;
    auto* ptr = static_cast<vtkDecimatePro*>(mpFilter.GetPointer());
    if (ptr->GetInput())
    {
      auto N_Old = dynamic_cast<vtkPolyData*>(ptr->GetInput())->GetNumberOfPoints();
      double reduction = std::min(1.0, 1.0 - (double)N / N_Old);
      ptr->SetTargetReduction(reduction);
    }
  }

  /**
  */
  void MeshReduction::update()
  {
    LOG_LINE << "start with points: " << mpInput->GetNumberOfPoints();
    removeSmallSubMeshes();
    mpFilter->SetInputData( mpWorker );
    auto* ptr = static_cast<vtkDecimatePro*>(mpFilter.GetPointer());
    auto N_Old = mpWorker->GetNumberOfPoints();
    double reduction = std::min(1.0, 1.0 - (double)mNumberOfPoints / N_Old);
    LOG_LINE << "reduction " << reduction;
    ptr->SetTargetReduction(reduction);
    mpFilter->SetInputData(mpWorker);
    mpFilter->Update();
    mpOutput->DeepCopy(mpFilter->GetOutput());
    LOG_LINE << "remaining points: " << mpFilter->GetOutput()->GetNumberOfPoints();
  }

  /** \brief compute regions with smaller than #mMinimumPoints vertices and remove them
  */
  void MeshReduction::removeSmallSubMeshes()
  {
    auto conn    = make_vtk<vtkPolyDataConnectivityFilter>();
    conn->SetExtractionModeToAllRegions();
    conn->SetInputData(mpInput);
    conn->Update();
    const auto N = conn->GetNumberOfExtractedRegions();
    const auto sizes = conn->GetRegionSizes();
    conn->InitializeSpecifiedRegionList(); // what that actually means is: clearSpecifiedRegionList();
    for (int i(0); i<N; ++i)
    {
      if (sizes->GetValue(i) >= (long long)mMinimumPoints)
      {
        conn->AddSpecifiedRegion(i);
      }
    }
    conn->SetExtractionModeToSpecifiedRegions();
    conn->Modified();
    conn->Update();
    auto cleaner = make_vtk<vtkCleanPolyData>();
    cleaner->SetInputData(conn->GetOutput());
    cleaner->Update();
    mpWorker = cleaner->GetOutput();
  }
}
}