#include "private/muk.pch"
#include "Remesher.h"

#include "AlgorithmFactory.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/vtk_tools.h"

#pragma warning (push)
#pragma warning( disable: 4996 ) // function (strcpy) may be unsafe
#include <vtkIdList.h>
#include <vtkIntArray.h>
#include <vtkIsotropicDiscreteRemeshing.h>
#include <vtkMultiThreader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkSurface.h>
#pragma warning (pop)

#include <array>

namespace
{
  /**
  */
  struct ClustersQuadrics
  {
      explicit ClustersQuadrics(int size) 
        : Elements(new double *[size])
        , Size(size)
      {
        for (int i = 0; i < size; ++i)
        {
          Elements[i] = new double[9];

          for (int j = 0; j < 9; ++j)
            Elements[i][j] = 0.0;
        }
      }

      ~ClustersQuadrics()
      {
        for (int i = 0; i < Size; ++i)
          delete[] Elements[i];
        delete Elements;
      }

      double **Elements;
      int Size;

    private:
      ClustersQuadrics(const ClustersQuadrics &);
      ClustersQuadrics &operator=(const ClustersQuadrics &);
  };
}

namespace gris
{
  namespace muk
  {
    REGISTER_ALGORITHM(Remesher);

    /**
    */
    Remesher::Remesher()
      : mNumberOfSamples(1000)
      , mGradation(0) // uniform
      , mSubsamplingThreshold(10) // default, higher -> better
      , mSplitLongMeshesRatio(0.0)
      , mpInput(nullptr)
    {
      mDspType = enDisplayPolyData;
      mInputPortTypes.push_back(enVtkPolyData);
      mOutputPortTypes.push_back(enVtkPolyData);

      mpOutput = make_vtk<vtkPolyData>();

      declareProperty<size_t>("NumberOfSamples",
        [=] (const auto val) { mNumberOfSamples = val; },
        [=] () { return mNumberOfSamples; });
      declareProperty<double>("Gradation",
        [=] (const auto val) { mGradation = val; },
      [=] () { return mGradation; });
      declareProperty<int>("Threshold",
        [=] (const auto val) { mSubsamplingThreshold = val; },
      [=] () { return mSubsamplingThreshold; });
      declareProperty<double>("SplitLongMeshesRatio",
        [=] (const auto val) { mSplitLongMeshesRatio = val; },
      [=] () { return mSplitLongMeshesRatio; });
    }

    /**
    */
    void Remesher::setInput(unsigned int portId, void* pDataType)
    {
      mpInput = toDataType<vtkPolyData>(pDataType);
    }

    /**
    */
    void* Remesher::getOutput(unsigned int portId)
    {
      return static_cast<void*>(mpOutput);
    }

    /**  
    */
    bool Remesher::hasBadInput()
    {
      if (nullptr == mpInput)
      {
        LOG_LINE << "Input surface is nullptr!";
        return true;
      }

      /*if (mpInput >= surface->GetSizeOfPolyDataSeries())
        mitkThrow() << "Input surface doesn't have data at time step " << t << "!";*/

      /*vtkPolyData *polyData = const_cast<mitk::Surface *>(surface.GetPointer())->GetVtkPolyData(t);

      if (mpInput == nullptr)
        mitkThrow() << "PolyData of input surface at time step " << t << " is nullptr!";
        */
      if (mpInput->GetNumberOfPolys() == 0)
      {
        LOG_LINE << "Input surface has no polygons!";
        return true;
      }
      return false;
    }

    /**  
    */
    void Remesher::update() 
    {
      ///*typedef vtkDiscreteRemeshing<vtkIsotropicMetricForClustering> IsotropicRemeshing;
      //typedef vtkDiscreteRemeshing<vtkQEMetricForClustering> QEMRemeshing;
      //typedef vtkDiscreteRemeshing<vtkQuadricAnisotropicMetricForClustering> QuadricAnisotropicRemeshing;*/
      //using AnisotropicRemeshing = vtkDiscreteRemeshing<vtkAnisotropicMetricForClustering> ;
      //auto Remesh= make_vtk<vtkVerticesProcessing<AnisotropicRemeshing>>();

      //auto mesh = make_vtk<vtkSurface>();
      //mesh->CreateFromPolyData(mpInput);
      //mesh->GetCellData()->Initialize();
      //mesh->GetPointData()->Initialize();
      //mesh->DisplayMeshProperties();
      //mesh->SplitLongEdges(mSplitLongMeshes);
      //// Remesh

      //Remesh->SetInput(mesh);
      //Remesh->SetNumberOfClusters(mNumberOfSamples);
      //Remesh->SetConsoleOutput(2);
      //Remesh->SetSubsamplingThreshold(mSubsamplingThreshold);
      //Remesh->GetMetric()->SetGradation(mGradation);
      //Remesh->Remesh();

      //auto* p = Remesh->GetOutput();
      //mpOutput->SetPoints(p->GetPoints());
      //mpOutput->SetPolys(p->GetPolys());
      //mpOutput->Modified();

      // ================================================================================================

      /*vtkSmartPointer<vtkPolyData> surfacePolyData = vtkSmartPointer<vtkPolyData>::New();
      surfacePolyData->DeepCopy(const_cast<Surface *>(surface.GetPointer())->GetVtkPolyData(t));*/

      if (hasBadInput())
      {
        mpOutput->Initialize();
        mpOutput->Modified();
        return;
      }

      auto mesh = make_vtk<vtkSurface>();
      mesh->CreateFromPolyData(mpInput);
      mesh->GetCellData()->Initialize();
      mesh->GetPointData()->Initialize();

      //mesh->DisplayMeshProperties();
      /*int numVertices;
      if (numVertices == 0)
        numVertices = surfacePolyData->GetNumberOfPoints();*/

      if (mSplitLongMeshesRatio!= 0.0)
        mesh->SplitLongEdges(mSplitLongMeshesRatio);

      auto remesher = make_vtk<vtkIsotropicDiscreteRemeshing>();

      const bool boundaryFixing = true;
      const bool forceManifold  = false;
      remesher->GetMetric()->SetGradation(mGradation);
      remesher->SetBoundaryFixing(boundaryFixing);
      remesher->SetForceManifold(forceManifold);
      remesher->SetInput(mesh);
      remesher->SetNumberOfClusters(mNumberOfSamples);
      //remesher->SetNumberOfThreads(vtkMultiThreader::GetGlobalDefaultNumberOfThreads());
      remesher->SetSubsamplingThreshold(mSubsamplingThreshold);
      remesher->SetConsoleOutput(0);

      remesher->Remesh();

      const int optimizationLevel = 0;

      // Optimization: Minimize distance between input surface and remeshed surface
      if (optimizationLevel != 0)
      {
        ClustersQuadrics clustersQuadrics(mNumberOfSamples);

        vtkSmartPointer<vtkIdList> faceList = vtkSmartPointer<vtkIdList>::New();
        vtkSmartPointer<vtkIntArray> clustering = remesher->GetClustering();
        vtkSmartPointer<vtkSurface> remesherInput = remesher->GetInput();
        int clusteringType = remesher->GetClusteringType();
        int numItems = remesher->GetNumberOfItems();
        int numMisclassifiedItems = 0;

        for (int i = 0; i < numItems; ++i)
        {
          int cluster = clustering->GetValue(i);

          if (cluster >= 0 && cluster < mNumberOfSamples)
          {
            if (clusteringType != 0)
            {
              remesherInput->GetVertexNeighbourFaces(i, faceList);
              int numIds = static_cast<int>(faceList->GetNumberOfIds());

              for (int j = 0; j < numIds; ++j)
                vtkQuadricTools::AddTriangleQuadric(
                  clustersQuadrics.Elements[cluster], remesherInput, faceList->GetId(j), false);
            }
            else
            {
              vtkQuadricTools::AddTriangleQuadric(clustersQuadrics.Elements[cluster], remesherInput, i, false);
            }
          }
          else
          {
            ++numMisclassifiedItems;
          }
        }

        if (numMisclassifiedItems != 0)
          std::cout << numMisclassifiedItems << " items with wrong cluster association" << std::endl;

        vtkSmartPointer<vtkSurface> remesherOutput = remesher->GetOutput();
        double point[3];

        for (int i = 0; i < mNumberOfSamples; ++i)
        {
          remesherOutput->GetPoint(i, point);
          vtkQuadricTools::ComputeRepresentativePoint(clustersQuadrics.Elements[i], point, optimizationLevel);
          remesherOutput->SetPointCoordinates(i, point);
        }

        std::cout << "After quadrics post-processing:" << std::endl;
        remesherOutput->DisplayMeshProperties();
      }

      vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();

      normals->SetInputData(remesher->GetOutput());
      normals->AutoOrientNormalsOn();
      normals->ComputeCellNormalsOff();
      normals->ComputePointNormalsOn();
      normals->ConsistencyOff();
      normals->FlipNormalsOff();
      normals->NonManifoldTraversalOff();
      normals->SplittingOff();

      normals->Update();
      mpOutput->DeepCopy(normals->GetOutput());
    }
  }
}