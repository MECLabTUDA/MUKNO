#pragma once

#include "muk_common_api.h"
#include "MukPath.h"
#include "LieGroupSE3.h"

#include "gstd/dynamicProperty.h"

namespace gris
{
  namespace muk
  {

    class MUK_COMMON_API IInterpolator : public gstd::DynamicProperty
    {
      public:
        enum EnInterpolationTypes
        {
          controlPoints,  // original path
          samples,        // e.g. computed waypoints for bezier-spirals
          pointsPerSegment,
          resolution,
          N_Types
        };

        static std::vector<std::string> getInterpolationTypes();
        static const char* InterpolationTypesAsStrings[N_Types];
        static int         typeFromString(const char*);

      public:        
        typedef std::shared_ptr<IInterpolator> Pointer;
        using result_type       = MukPath;
        using InterpolatedPoses = std::vector<se3::Pose3D>;

      public:
        IInterpolator();
        IInterpolator(const IInterpolator&)            = delete;
        IInterpolator& operator=(const IInterpolator&) = delete;
        IInterpolator(IInterpolator&& o);
        IInterpolator& operator=(IInterpolator&&);
        virtual ~IInterpolator() {}
        
      public:        
        virtual const char* name() const = 0;
        
      public:
        virtual void    setKappa(double kappa)                  { mKappa = kappa; }
        virtual double  getKappa()                  const       { return mKappa;  }

        void            setInput(const MukPath& p);
        const MukPath&  getInput()                  const       { return mInput;  }
        void            setResolution(double val)               { mResolution = val; }
        double          getResolution()             const       { return mResolution; }
        void            setPointsPerSegment(size_t val)         { mPointsPerSegment = val; }
        size_t          getPointsPerSegment()       const       { return mPointsPerSegment; }
        void                  setInterpolationType(EnInterpolationTypes val)           { mInterpolationType = val;  }
        EnInterpolationTypes  getInterpolationType() const                             { return mInterpolationType; }
        
        virtual void    clone(const IInterpolator* pOther);
        
        virtual void    interpolate() = 0;

      public:
        virtual bool isValid()                    const = 0;

      public:
        result_type       getInterpolation()     const;
        InterpolatedPoses getInterpolatedPoses() const;
        virtual result_type getControlPoints() const = 0;
        virtual std::vector<bool> validStates() const = 0;

      protected:
        virtual result_type       getInterpolation(EnInterpolationTypes)     const = 0;
        virtual InterpolatedPoses getInterpolatedPoses(EnInterpolationTypes) const;
        void appendProperties();
        
      protected:
        double  mKappa;
        MukPath mInput;
        bool    mInterpolated;

        size_t  mPointsPerSegment;
        double  mResolution;
        EnInterpolationTypes mInterpolationType;
    };

  }
}

namespace std
{
  MUK_COMMON_API std::ostream& operator<< (std::ostream& os, const gris::muk::IInterpolator::EnInterpolationTypes& obj); 
  MUK_COMMON_API std::istream& operator>> (std::istream& is, gris::muk::IInterpolator::EnInterpolationTypes& obj);
}
