#pragma once

#include "muk_common_api.h"

#include "gstd/dynamicProperty.h"

#include "MukCommon/MukPath.h"
#include "MukCommon/MukVector.h"
#include "MukCommon/MukState.h"
#include "MukCommon/MukException.h"
#include "MukCommon/INavigationSupervisor.h"
#include "MukCommon/CalibrationStateMachine.h"

#include "MukCommon/IInterpolator.h"

#include <memory>
#include <mutex>
#include <atomic>
#include <functional>
#include <set>

namespace gris
{
  namespace muk
  {

    class INavigator;

    // adds the NavigatorFeatures to the Properties
    class MUK_COMMON_API NavigatorProperty : public gstd::DynamicProperty
    {
    public:
      NavigatorProperty(INavigator* nav);
      NavigatorProperty(const NavigatorProperty& rhs);
      NavigatorProperty(NavigatorProperty&& rhs);
      ~NavigatorProperty() {}
    protected:
      NavigatorProperty() : gstd::DynamicProperty() {};
    };

    class MUK_COMMON_API INavigator : public TimerOwner
    {
    public:
      class MUK_COMMON_API PathHelper
      {
      public:
        typedef std::map<const double, const MukState&> DistanceMap;

      public:
        PathHelper(std::unique_ptr<IInterpolator>& pInterpolator);
        void              calculateDistances();
        const DistanceMap getDistanceMap() const;
        MukState          interpolate(const double& t) const;
        double            findNearestPositionOnPath(const Vec3d& pos);

      private:
        DistanceMap       mDistanceMap;
        std::unique_ptr<IInterpolator>& mpInterpolator;
        MukPath           mPath;
      };

    public:
      typedef std::vector<std::string> CoordinateSystemList;

    public:
      enum NavigatorFeature
      {
        invalid,
        updateCoordinateSystem,
        updateRobotTransform,
        updateState,
        updateAdaptedPath,
        updateCurrentTargetOrientation,

        debugMessages
      };
      static char* NavigatorFeatureToString(const NavigatorFeature& f);
      static NavigatorFeature StringToNavigatorFeature(const std::string& str);
      // value 0 is "invalid" and not included ...
      static inline std::size_t NavigatorFeatureCount() { return 6; }

    private:
      static const double POSITION_RESOLUTION_FACTOR;


    public:
      INavigator();
      virtual ~INavigator();

    public:
      virtual const char* name() const { return "Navigator"; }

    public:
      void                    setSupervisor(INavigationSupervisor* supervisor)    { mpSupervisor = supervisor; }
      INavigationSupervisor*  getSupervisor()                               const { return mpSupervisor; }
      // sets the Path and prepares the Interpolator
      virtual void            setPath(const MukPath& path);
      const MukPath&          getPath()                                     const { return mpInterpolator->getInput(); }
      IInterpolator*          getInterpolator()                             const;
      virtual void            assumeInterpolator(std::unique_ptr<IInterpolator>& pInterpolator);
      void                    assumeInterpolatorNoPrepare(std::unique_ptr<IInterpolator>& pInterpolator);
      void                    setInitialResolution(const double resolution);
      void                    fixMinimumResolution();
      
    public:
      virtual void            initCalibrator(CalibrationStateMachine&);

    protected:
      virtual MukState        interpolate(const double& t) const;
      double                  findNearestPositionOnPath(const Vec3d& pos);
      void                    prepareInterpolator();

    public:
      /**
      * init() initializes the Navigator.
      * This can, but does not have to be the actual initialization step.
      */
      virtual void  init() = 0;
      /**
      * start() starts delivering Position updates to the "Main Program".
      * It can be expected, that the Navigator was initialized before.
      * This is not the actual delivery of the positions, use runContinuous() or a TimerEvent for that.
      */
      virtual void  start() = 0;
      /**
      * stop() stops the delivery of Position updates to the "Main Programm".
      * It can be expected, that the Navigator was initialized before and the Navigator should be initialized after.
      */
      virtual void  stop() = 0;
      /**
      * proceed() restarts the delivery of Position updates to the "Main Programm".
      * It can be expected, that the Navigator was initialized before and the Navigator should be initialized after.
      */
      virtual void  proceed() = 0;
      /**
      * terminate() stops the delivery of Position updates to the "Main Programm" and stops the execution 
      * of ANY Navigator calculations (the Navigator should be fully stopped using this).
      * The Navigator does not have to be initialized after.
      */
      virtual void  terminate() = 0;

      /**
      * state() returns the Navigator State.
      */
      virtual enNavigatorState state() const;

    public:
      void          setNavigatorFeature(const NavigatorFeature, const bool);
      bool          isFeatureActivated(const NavigatorFeature) const;
      void          setCoordinateSystems(const CoordinateSystemList&);
      void          setCoordinateSystems(const std::string&);
      const CoordinateSystemList& getCoordinateSystems() const;
      std::string   stringifyCoordinateSystems() const;

      void          setPositionThreshold(const double threshold);
      double        positionThreshold() const;
      void          setOrientationThreshold(const double threshold);
      double        orientationThreshold() const;

    protected:
      void          setState(const enNavigatorState&);

    public:
      virtual std::unique_ptr<NavigatorProperty>     getProperty() { return std::make_unique<NavigatorProperty>(this); }

    public:
      // continuous Run Mode
      // returns true, if this "Navigator" should be terminated (i.e. reset)
      // this function should not change the current Navigator!
      virtual bool      runContinuous() = 0;

    private:
      INavigationSupervisor*         mpSupervisor;
      std::unique_ptr<IInterpolator> mpInterpolator;
      PathHelper                     mPathHelper;

      std::atomic<enNavigatorState>  mState;

      double                         mPositionThreshold;
      double                         mOrientationThreshold;
      double                         mInitialResolution;

      std::set<NavigatorFeature>     mActivatedFeatures;
      CoordinateSystemList           mCoordinateSystems;
    };

    class MUK_COMMON_API ProtectedNavigator
    {
    public:
      ProtectedNavigator(INavigator* const pNavigatorToProtect, std::mutex& mutex);
      ProtectedNavigator(const ProtectedNavigator& pn) : mpNavigator(pn.mpNavigator), mMutex(pn.mMutex) {};

    public:
      virtual const char* name() const { return mpNavigator->name(); }

    public:
      std::unique_ptr<IInterpolator> getInterpolator()                             const;

    public:
      virtual enNavigatorState state()     const { return mpNavigator->state(); }

    protected:
      void        setNavigator(INavigator* pObj)         { mpNavigator = pObj; }
      void        setNavigator(ProtectedNavigator* pObj) { setNavigator(pObj->getNavigator());  }
      INavigator* getNavigator() const                   { return mpNavigator; };

    private:
      INavigator* mpNavigator;
      std::mutex& mMutex;

    };

  }
}