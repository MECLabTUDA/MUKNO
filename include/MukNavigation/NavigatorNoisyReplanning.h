#pragma once
#include "NavigatorDummy.h"

#include <random>

namespace gris
{
  namespace muk
  {
    class NavigatorNoisyReplanning;
    
    /**
    */
    class NavigatorNoisyReplanningProperty : public NavigatorDummyProperty
    {
      public:
        NavigatorNoisyReplanningProperty(NavigatorNoisyReplanning* nav);
    };

    /** \brief Simulates the navigation with noisy data or slight misalignments

      Each timestep a small gaussian noise is added to the current position and orientation.
    */
    class MUK_NAVI_API NavigatorNoisyReplanning : public NavigatorDummy
    {
      public:
        NavigatorNoisyReplanning();

      public:
        static  const char* s_name()        { return "NavigatorNoisyReplanning"; }
        virtual const char* name()    const { return s_name();  }
        
      protected:
        virtual std::unique_ptr<NavigatorProperty> getProperty();
        virtual MukState currentState(); // here noisy stuff

        void resetRandomWalk();

      public:
        virtual void setPath(const MukPath & path);
        virtual void init();
        virtual void start();
        virtual void preReplanning();
        virtual void postReplanning();

      public:
        void   setTranslationalMean                (double val) { mTranslationalMean = val; } 
        void   setTranslationalVariance            (double val) { mTranslationalVariance = val; } 
        void   setTranslationalMeasurementVariance (double val) { mTranslationalMeasurementVariance = val; }
        void   setRotationalMean                   (double val) { mRotationalMean = val; } 
        void   setRotationalVariance               (double val) { mRotationalVariance = val; } 
        void   setRotationalMeasurementVariance    (double val) { mRotationalMeasurementVariance = val; }

        double  getTranslationalMean               () const { return mTranslationalMean; } 
        double  getTranslationalVariance           () const { return mTranslationalVariance; }
        double  getTranslationalMeasurementVariance() const { return mTranslationalMeasurementVariance; }
        double  getRotationalMean                  () const { return mRotationalMean; }
        double  getRotationalVariance              () const { return mRotationalVariance; }
        double  getRotationalMeasurementVariance   () const { return mRotationalMeasurementVariance; }

      protected:
        Vec3d  addRotate(const Vec3d& left, const Vec3d& right);


        bool   mRecordMeasurementNoise;
        Vec3d  mMeasurementNoiseT;
        Vec3d  mMeasurementNoiseR;

        size_t mAccumulatedMeasurements;
        Vec3d  mAccumulatedNoiseT;
        Vec3d  mAccumulatedNoiseR;
        double mTranslationalMean;
        double mTranslationalVariance;
        double mTranslationalMeasurementVariance;
        double mRotationalMean;
        double mRotationalVariance;
        double mRotationalMeasurementVariance;
        std::random_device mRandomDevice;
        std::mt19937       mMersenne;
    };

  }
}