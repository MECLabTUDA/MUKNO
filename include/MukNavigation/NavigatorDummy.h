#pragma once

#include "muk_navigation_api.h"
#include "MukCommon/INavigator.h"
#include "MukCommon/INavigationSupervisor.h"

#include <chrono>

namespace gris
{
  namespace muk
  {
    class NavigatorDummy;

    class NavigatorDummyProperty : public NavigatorProperty
    {
    public:
      NavigatorDummyProperty(NavigatorDummy* nav);
    };

    /**
    */
    class MUK_NAVI_API NavigatorDummy : public INavigator
    {
    public:
      // update interval in Milliseconds
      static const std::chrono::milliseconds UPDATE_INTERVAL;
      static const std::chrono::duration<double, std::milli> SPEED_INV;

      typedef std::chrono::duration<double> seconds;

    public:
      NavigatorDummy();
      virtual ~NavigatorDummy() { } // no actual resources are inn control of this class

    public:
      static const char* s_name()       { return "NavigatorDummy"; }
      virtual const char* name()  const { return s_name();  }
       
    private:
      void tick(INavigationSupervisor::TimerID id);
      void adaptPath(INavigationSupervisor::TimerID id);
      void proceedImpl();
      void haltMovement();

    public:
      virtual void init();
      virtual void start();
      virtual void stop();
      virtual void proceed();
		  virtual bool runContinuous();
      virtual void terminate();
      virtual void preReplanning();
      virtual void postReplanning();

      // TODO @David: also check, whether our Timers are still intact
      // virtual bool isInitialized() const;
      // virtual bool isRunning() const;

    protected:
      virtual MukState currentState();

    public:
      virtual std::unique_ptr<NavigatorProperty> getProperty();

      void setSpeed(const double speed);
      double getSpeed() const;

      virtual void setPath(const MukPath& path);

      void setUpdateInterval(const std::chrono::milliseconds::rep interval);
      std::chrono::milliseconds::rep getUpdateInterval() const;
      void setExportFilename(const std::string& interval);
      std::string getExportFilename() const;
    private:
      void openfile();

    private:
      std::chrono::nanoseconds mOffset;
      std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> mStartTime;
      std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> mPauseTime;
      // bool mTerminateFlag

      std::chrono::duration<double, std::milli> mSpeed_inv;

      std::string               mExportFilename;
      std::chrono::milliseconds mUpdateInterval;
      std::ofstream             mExportStream;

      MukState                  mReplanningStateBuffer;
    };

  }
}