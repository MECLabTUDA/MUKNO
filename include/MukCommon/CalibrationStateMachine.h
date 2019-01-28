#pragma once

#include "muk_common_api.h"

// MukCommon
#include "INavigationSupervisor.h"

// VTK
#include "vtkSmartPointer.h"
#include "vtkXMLDataElement.h"

#include <vector>
#include <functional>


class vtkPlusTransformRepository;
enum PlusStatus;

namespace gris
{
namespace muk
{
  class MUK_COMMON_API CalibrationStateMachine : public TimerOwner
  {
  public:
    typedef std::function<void(vtkSmartPointer<vtkPlusTransformRepository>)>  CallbackFunction;
    typedef std::function<vtkSmartPointer<vtkPlusTransformRepository>()>      TransformGeneratorFunction;
    typedef std::function<void()>                                             FinishedFunction;

  public:
    CalibrationStateMachine();
    ~CalibrationStateMachine();

  public:
    static  const char* s_name()       { return "StateMachine"; }
    virtual const char* name()   const { return s_name(); }

    static  const char* s_printableName()        { return "State Machine"; }
    virtual const char* printableName()    const { return s_printableName(); }

  public:
/*    void setSourceTool(const std::string& str) { mSourceTool = str; }
    std::string getSourceTool() const { return mSourceTool; }
    void setTargetTool(const std::string& str) { mTargetTool = str; }
    std::string getTargetTool() const { return mTargetTool; }*/
    void setSupervisor(INavigationSupervisor* pSup);
    INavigationSupervisor* getSupervisor() const;
    void setTransformGenerator(const TransformGeneratorFunction& tg);
    const TransformGeneratorFunction& getTransformGenerator() const;
    void setFinisher(const FinishedFunction& ff);
    const FinishedFunction& getFinisher() const;
    bool isFinished() const;
    unsigned int getCountDownTicks() const { return mCountdownTicks; }
    void setCountDownTicks(const unsigned int v) { mCountdownTicks = v; }
    void setCountDownTicks(const int v) { setCountDownTicks(v > 0 ? unsigned int(v) : 1); }

  public:
    void registerCalibrationStep(CallbackFunction cbFunction);
    void registerCalibrationStep(const std::string& message);
    void finishStep(const std::string& message, bool waitProceed = true);
    virtual PlusStatus ReadConfiguration(vtkSmartPointer<vtkXMLDataElement> elem);

    void tick(INavigationSupervisor::TimerID timerid);
    virtual void init(); // also used to reset
    virtual void proceed();
    virtual void terminate();

  public:
    enConsumerState state() const { return mState; }
    bool isWaiting() const { return mWaiting; }
  protected:
    void setState(const enConsumerState);

  public:
    const std::chrono::milliseconds& getTimerInterval() const;

  private:
    void countDownToStep();
    void initializeTimer();

  private:
    INavigationSupervisor*                  mpParent;
    bool                                    mWaiting;
    INavigationSupervisor::TimerID          mTimerID;
    std::chrono::milliseconds               mTimerInterval;

    unsigned int                            mCountdownTicks;

/*    std::string                             mSourceTool;
    std::string                             mTargetTool;*/

    std::vector<CallbackFunction>           mCalibrationFunctions;
    std::vector<CallbackFunction>::iterator mStepIterator;
    TransformGeneratorFunction              mTransformGenerator;

    FinishedFunction                        mFinisher;
    std::atomic<enConsumerState>            mState;
  };
}
}