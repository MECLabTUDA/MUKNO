#include "private/muk.pch"
#include "DeformableMesh.h"
#include "AlgorithmFactory.h"
#include "muk_algorithms_tools.h"

#include "MukCommon/muk_common.h"

#include "MukImaging/MukImage.h"
#include "MukImaging/muk_imaging_tools.h"

#include <itkSimplexMesh.h>
#include <itkSimplexMeshToTriangleMeshFilter.h>
#include <itkTriangleMeshToSimplexMeshFilter.h>

#include <itkDeformableSimplexMesh3DBalloonForceFilter.h>
#include <itkDeformableSimplexMesh3DFilter.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(DeformableMesh);

  struct DeformableMesh::Impl
  {
    vtkPolyData*      inputMesh;
    GradientImage3D* gradient;
    vtkSmartPointer<vtkPolyData> outputMesh;

    // from https://itk.org/Doxygen/html/classitk_1_1DeformableSimplexMesh3DFilter.html
    double alpha = 0.2; // The interal force can be scaled via SetAlpha (typical values are 0.01 < alpha < 0.3)
    double beta  = 0.01; // The external forces are scaled via SetBeta (typical values are 0.01 < beta < 1).
    double gamma = 0.05; // controls regularity of the mesh. Low values (< 0.03) produce more regular mesh. Higher values ( 0.3 < gamma < 0.2) will allow to move the vertices to regions of higher curvature. 
    int    iterations = 1;
    int    rigidity = 1; // smoothness of the mesh. Low values (1 or 0) allow areas with high curvature. Higher values (around 7 or 8) will make the mesh smoother.
    double kappa = 1.0;
  };

  /** 
  */
  DeformableMesh::DeformableMesh()
    : mp (std::make_unique<Impl>())
  {
    mDspType = enDisplayPolyData;
    mInputPortTypes.push_back(enVtkPolyData);
    mOutputPortTypes.push_back(enVtkPolyData);

    mp->outputMesh = make_vtk<vtkPolyData>();
    //auto* ptr = dynamic_cast<Type*>(mpFilter.GetPointer());

    declareProperty<double>("Alpha",
      [=] (auto val) { mp->alpha = val; },
      [=] ()         { return mp->alpha; });
    declareProperty<double>("Beta",
      [=] (auto val) { mp->beta = val; },
      [=] ()         { return mp->beta; });
    declareProperty<double>("Gamma",
      [=] (auto val) { mp->gamma = val; },
      [=] ()         { return mp->gamma; });
    declareProperty<int>("Iterations",
      [=] (auto val) { mp->iterations = val; },
      [=] ()         { return mp->iterations; });
    declareProperty<int>("Rigidity",
      [=] (auto val) { mp->rigidity = val;},
      [=] ()         { return mp->rigidity; });/*
    declareProperty<double>("Kappa",
      [=] (auto val) { mp->kappa = val; },
      [=] ()         { return mp->kappa; });*/
  }

  /**
  */
  void DeformableMesh::setInput(unsigned int portId, void* pDataType)
  {
    switch (portId)
    {
      case 0:
        mp->inputMesh = toDataType<vtkPolyData>(pDataType);
        break;
      case 1:
        mp->gradient  = toDataType<GradientImage3D>(pDataType);
        break;
    };
  }

  /**
  */
  void* DeformableMesh::getOutput(unsigned int portId)
  {
    return static_cast<void*>(mp->outputMesh.GetPointer());
  }

  /** \brief check if an update is needed, then create the pipeline and copy the result

    the inputMesh mesh is converted from vtkPolyData to a itk::Mesh (TriangleMesh). This is converted to
    a simplex mesh and them given as input to the Deformable Mesh Filter.
    Its result is converted the same way back, first to a triangle mehs, then to vtkPolyData.
  */
  void DeformableMesh::update()
  {
    //using TriangleMesh      = itk::Mesh<double, ImageInt3DDim>;
    using SimplexMesh       = itk::SimplexMesh<MukFloatPixel, Dim3D, MeshTrait>;
    using TriangleToSimplex = itk::TriangleMeshToSimplexMeshFilter<TriangleMesh, SimplexMesh>;
    using SimplexToTriangle = itk::SimplexMeshToTriangleMeshFilter<SimplexMesh, TriangleMesh>;
    using DeformableFilter            = itk::DeformableSimplexMesh3DFilter<SimplexMesh, SimplexMesh>;
    using DeformableBallooForceFilter = itk::DeformableSimplexMesh3DBalloonForceFilter<SimplexMesh, SimplexMesh>;
    
    auto triangleFilter = make_vtk<vtkTriangleFilter>();
    triangleFilter->SetInputData(mp->inputMesh);
    triangleFilter->SetPassLines(0);
    triangleFilter->SetPassVerts(0);
    triangleFilter->Update();
    auto sphere = triangleFilter->GetOutput();
    auto mesh = make_itk<TriangleMesh>();
    polyDataToItkMesh(*sphere, *mesh);

    // convert Mesh to simplemesh	 
    auto simplexFilter = make_itk<TriangleToSimplex>();
    simplexFilter->SetInput(mesh);
    //simplexFilter->Update(); // this breaks the code and results in a crash in the simplex-to-triangle conversion below!

    auto deformableMesh = make_itk<DeformableFilter>();
    {
      deformableMesh->SetInput(simplexFilter->GetOutput());
      deformableMesh->SetGradient(mp->gradient);
      deformableMesh->SetAlpha(mp->alpha);
      deformableMesh->SetBeta(mp->beta);
      deformableMesh->SetGamma(mp->gamma);
      //deformableMesh->SetKappa(mp->kappa);
      deformableMesh->SetIterations(mp->iterations);
      deformableMesh->SetRigidity(mp->rigidity);
    }

    auto toTriangle = make_itk<SimplexToTriangle>();
    toTriangle->SetInput(deformableMesh->GetOutput());
    toTriangle->Update();

    auto poly = make_vtk<vtkPolyData>();
    itkMeshToVtkPolyData(*toTriangle->GetOutput(), *poly);

    mp->outputMesh->DeepCopy(poly);
  }
} // namespace muk
} // namespace gris