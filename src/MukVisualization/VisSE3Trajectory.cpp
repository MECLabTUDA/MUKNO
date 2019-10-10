#include "private/muk.pch"
#include "VisSE3Trajectory.h"
#include "MukVtkAxesActor.h"

#include "MukCommon/vtk_tools.h"
#include "MukCommon/muk_dynamic_property_tools.h"

#include <vtkAxesActor.h>
#include <vtkRenderer.h>

namespace gris
{
namespace muk
{
  /**
  */
  struct VisSE3Trajectory::Impl
  {
    Impl(VisSE3Trajectory* p)
      : parent(p)
    {
    }

    void setScale(double d)
    {
      for(size_t i(0); i<trafos.size(); ++i)
      {
        auto& trafo  = trafos[i];
        // we have to create a new one, otherwise vtk starts crying
        auto T = make_vtk<vtkTransform>();
        auto* matrix = trafo->GetMatrix();
        double elements[16];
        for (int i(0); i < 4; ++i)
          for (int j(0); j < 4; ++j)
            elements[4 * i + j] = matrix->GetElement(i, j);
        for (int i(0); i<3; ++i)
          for (int j(0); j<3; ++j)
          {
            elements[4*i + j] *= 1.0/ scaleFactor * d;
          }
        T->SetMatrix(elements);
        trafos[i] = T;
        poses[i]->SetUserTransform(T);
        poses[i]->Modified();
      }
      scaleFactor = d;
    }
    double getScale() const { return scaleFactor; }
    
    VisSE3Trajectory* parent;
    double scaleFactor = 1.0;
    std::vector<vtkSmartPointer<vtkTransform>> trafos;
    std::vector<vtkSmartPointer<MukVtkAxesActor>> poses;
  };

  // ------------------------------------------------------------

  /**
  */
  VisSE3Trajectory::VisSE3Trajectory(const std::string& name)
    : VisAbstractObject(name)
    , mp(std::make_unique<Impl>(this))
  {
    declareProperty<double>("Scale", [&](double d) { mp->setScale(d); }, [&]() { return mp->getScale(); });
  }

  /**
  */
  VisSE3Trajectory::~VisSE3Trajectory()
  {
    if (mpRenderer)
    {
      std::for_each(mp->poses.begin(), mp->poses.end(), [&] (auto& prop) { mpRenderer->RemoveViewProp(prop); } );
    }
  }

  /**
  */
  void VisSE3Trajectory::update()
  {
  }

  /**
  */
  void VisSE3Trajectory::setNumberOfStates(size_t n)
  {
    // delete old stuff
    if (mpRenderer)
    {
      std::for_each(mp->poses.begin(), mp->poses.end(), [&] (auto& prop) { mpRenderer->RemoveViewProp(prop); } );
    }
    // rebuild and copy the actors
    mp->poses.resize(n);
    mp->trafos.resize(n);
    for(size_t i(0); i<n; ++i)
    {
      mp->trafos[i]= make_vtk<vtkTransform>();
      mp->poses[i] = make_vtk<MukVtkAxesActor>();
      //mp->poses[i]->SetAxisLabels(0);
      mp->poses[i]->SetScale(mp->scaleFactor);
      mp->poses[i]->SetUserTransform(mp->trafos[i]);
      if (mpRenderer)
        mpRenderer->AddViewProp(mp->poses[i]);
    }
  }

  /**
  */
  void VisSE3Trajectory::setPose(size_t i, const se3::Pose3D& pose)
  {
    double d[16];
    for (size_t i(0); i < 4; ++i)
      for (size_t j(0); j < 4; ++j)
        d[4 * i + j] = pose.data()(i, j);
    mp->trafos[i]->SetMatrix(d);
    mp->trafos[i]->Modified();
    mp->poses[i]->Modified();
  }

  /**
  */
  void VisSE3Trajectory::setPoses(const std::vector<se3::Pose3D>& poses)
  {
    setNumberOfStates(poses.size());
    for (size_t i(0); i < poses.size(); ++i)
      setPose(i, poses[i]);
  }
}
}