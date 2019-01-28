#include "private/muk.pch"
#include "MeshSource.h"
#include "AlgorithmFactory.h"

#include "MukCommon/muk_common.h"

#include <vtkPolyData.h>
#include <vtkSphereSource.h>

namespace gris
{
namespace muk
{
  REGISTER_ALGORITHM(MeshSource);

  /**
  */
  struct MeshSource::Impl
  {
    vtkSmartPointer<vtkPolyData> output;
    vtkSmartPointer<vtkSphereSource> sphere;
    bool modified = true;
  };

  /**
  */
  MeshSource::MeshSource()
    : mp (std::make_unique<Impl>())
  {
    mDspType = enDisplayPolyData;
    mOutputPortTypes.push_back(enVtkPolyData);

    mp->output = make_vtk<vtkPolyData>();
    mp->sphere = make_vtk<vtkSphereSource>();

    declareProperty<double>("Resolution",
      [&] (double val)  { mp->sphere->SetThetaResolution(val); mp->sphere->SetPhiResolution(val); mp->modified = true;},
      [&] ()            { return mp->sphere->GetThetaResolution(); });
    declareProperty<double>("Radius",
      [&] (double val)  { mp->sphere->SetRadius(val); mp->modified = true;},
      [&] ()            { return mp->sphere->GetRadius(); });
    declareProperty<Vec3d>("Center",
      [&] (const Vec3d& p) { mp->sphere->SetCenter(p.x(), p.y(), p.z()); mp->modified = true;},
      [&] ()               { Vec3d p; mp->sphere->GetCenter(p.data()); return p; });
  }

  /**
  */
  void* MeshSource::getOutput(unsigned int portId)
  {
    return static_cast<void*>(mp->output);;
  }
    
  /**
  */
  void MeshSource::update()
  {
    if (mp->modified)
    {
      mp->sphere->Update();
      mp->output->DeepCopy(mp->sphere->GetOutput());
    }
  }
}
}
