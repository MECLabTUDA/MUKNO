#include "private/muk.pch"
#include "NavigatorNoisyReplanning.h"

#include "MukCommon/NavigatorFactory.h"

#include "vtkTransform.h"
#include "vtkMatrix3x3.h"

#include <memory>


typedef vtkSmartPointer<vtkTransform> Transform;

namespace
{
  Transform make_transform(gris::Vec3d v)
  {
    // prepare pL and pR
    Transform p = Transform::New();
    p->RotateZ(v.z());
    p->RotateX(v.x());
    p->RotateY(v.y());
    return p;
  }
}


namespace gris
{
  namespace muk
  {
    REGISTER_NAVIGATOR(NavigatorNoisyReplanning);

    /**
    */
    NavigatorNoisyReplanningProperty::NavigatorNoisyReplanningProperty(NavigatorNoisyReplanning* nav)
      : NavigatorDummyProperty(nav)
    {
      declareProperty<double>("T_Mean_Gaussian_Noise"
        , [=] (const auto val)  { nav->setTranslationalMean(val); }
        , [=] ()                { return nav->getTranslationalMean(); });
      declareProperty<double>("T_Variance_Gaussian_Noise"
        , [=] (const auto val)  { nav->setTranslationalVariance(val); }
        , [=] ()                { return nav->getTranslationalVariance(); });
      declareProperty<double>("T_Measurement_Gaussian_Noise"
        , [=] (const auto val)  { nav->setTranslationalMeasurementVariance(val); }
        , [=] ()                { return nav->getTranslationalMeasurementVariance(); });
      declareProperty<double>("R_Mean_Gaussian_Noise"
        , [=] (const auto val)  { nav->setRotationalMean(val); }
        , [=] ()                { return nav->getRotationalMean(); });
      declareProperty<double>("R_Variance_Gaussian_Noise"
        , [=] (const auto val)  { nav->setRotationalVariance(val); }
        , [=] ()                { return nav->getRotationalVariance(); });
      declareProperty<double>("R_Measurement_Gaussian_Noise"
        , [=] (const auto val)  { nav->setRotationalMeasurementVariance(val); }
        , [=] ()                { return nav->getRotationalMeasurementVariance(); });
    }

    /**
    */
    NavigatorNoisyReplanning::NavigatorNoisyReplanning()
      : NavigatorDummy()
      , mMersenne( mRandomDevice() )
      , mTranslationalMean(0.0)
      , mTranslationalVariance(0.015)
      , mTranslationalMeasurementVariance(0.025)
      , mRotationalMean(0.0)
      , mRotationalVariance(0.5)
      , mRotationalMeasurementVariance(0.2)
      , mAccumulatedMeasurements(0)
      , mAccumulatedNoiseR(0,0,0)
      , mAccumulatedNoiseT(0,0,0)
      , mMeasurementNoiseR(0,0,0)
      , mMeasurementNoiseT(0,0,0)
    {
    }
    
    /**
    * only update the random walk, if the Navigator is running.
    */
    MukState NavigatorNoisyReplanning::currentState()
    {
      if (state() == Running)
      {
        ++mAccumulatedMeasurements;
        std::normal_distribution<> position_noise(mTranslationalMean, mTranslationalVariance);
        for (size_t i(0); i < 3; ++i)
          mAccumulatedNoiseT[i] += position_noise(mMersenne);
        std::normal_distribution<> rotation_noise(mRotationalMean, mRotationalVariance);
        Vec3d rNoise;
        for (size_t i(0); i < 3; ++i)
          rNoise[i] = rotation_noise(mMersenne);
        mAccumulatedNoiseR = addRotate(mAccumulatedNoiseR, rNoise);
      }
      std::normal_distribution<> measure_position_noise(0.0, mTranslationalMeasurementVariance);
      std::normal_distribution<> measure_rotation_noise(0.0, mRotationalMeasurementVariance);

      auto state = NavigatorDummy::currentState();
      
      Vec3d measure_pos_n, measure_rot_n;
      for (size_t i(0); i < 3; ++i)
        measure_pos_n[i] += measure_position_noise(mMersenne);
      for (size_t i(0); i < 3; ++i)
        measure_rot_n[i] += measure_rotation_noise(mMersenne);

      if (mRecordMeasurementNoise)
      {
        mMeasurementNoiseR = measure_rot_n;
        mMeasurementNoiseT = measure_pos_n;
      }

      state.coords  += measure_pos_n + mAccumulatedNoiseT;
      auto t = make_transform(addRotate(mAccumulatedNoiseR, measure_rot_n));
      state.tangent = Vec3d(t->TransformDoubleNormal(state.tangent.data()));
      state.tangent.normalize();
      return state;
    }

    /**
    */
    void NavigatorNoisyReplanning::resetRandomWalk()
    {
      mAccumulatedNoiseT = Vec3d(0, 0, 0);
      mAccumulatedNoiseR = Vec3d(0, 0, 0);
      mMeasurementNoiseT = Vec3d(0, 0, 0);
      mMeasurementNoiseR = Vec3d(0, 0, 0);
    }

    /**
    * reset random walk on setPath
    */
    void NavigatorNoisyReplanning::setPath(const MukPath & path)
    {
      NavigatorDummy::setPath(path);
    }

    /**
    * reset random walk noise on initialization
    */
    void NavigatorNoisyReplanning::init()
    {
      NavigatorDummy::init();
      resetRandomWalk();
    }

    void NavigatorNoisyReplanning::start()
    {
      resetRandomWalk();
      NavigatorDummy::start();
    }

    void NavigatorNoisyReplanning::preReplanning()
    {
      mRecordMeasurementNoise = false;
    }

    void NavigatorNoisyReplanning::postReplanning()
    {
      mRecordMeasurementNoise = true;
      auto t = Vec3d(0,0,0)-mMeasurementNoiseT;
      auto r = Vec3d(0,0,0)-mMeasurementNoiseR;
      resetRandomWalk();
      mAccumulatedNoiseT = t;
      mAccumulatedNoiseR = r;
    }

    Vec3d NavigatorNoisyReplanning::addRotate(const Vec3d & left, const Vec3d & right)
    { // checked in TryAnything
      // VTK: ZXY order
      // prepare pL and pR
      auto pL = make_transform(left), pR = make_transform(right);
      // Concat
      pL->PreMultiply();
      pL->Concatenate(pR);
      // return angles
      return Vec3d(pL->GetOrientation());
    }

    /**
    */
    std::unique_ptr<NavigatorProperty> NavigatorNoisyReplanning::getProperty()
    {
      return std::make_unique<NavigatorNoisyReplanningProperty>(this);
    }
  }
}