#include "private/muk.pch"
#include "CalibrationStateMachine.h"

#define BOOST_CHRONO_DONT_PROVIDES_DEPRECATED_IO_SINCE_V2_0_0
#include <boost/chrono/chrono_io.hpp>
#include <boost/format.hpp>

namespace gris
{
namespace muk
{

  /**
  */
  CalibrationStateMachine::CalibrationStateMachine()
    : mpParent(nullptr)
    , mTimerID(0)
    , mWaiting(false)
    , mTimerInterval(1000)
    , mTransformGenerator(nullptr)
    , mCountdownTicks(5)
    , mState(Invalid)
    , mStepIterator(mCalibrationFunctions.end())
/*    , mSourceTool("CalibrationSensor")
    , mTargetTool("RobotSensor")*/
  {
    registerCalibrationStep(std::string("Prepare for ") + printableName());
  }

  /**
  */
  CalibrationStateMachine::~CalibrationStateMachine()
  {
  }

  /**
  */
  void CalibrationStateMachine::registerCalibrationStep(CallbackFunction cbFunction)
  {
    bool atStart = mCalibrationFunctions.begin() == mStepIterator;
    bool atEnd = mCalibrationFunctions.end() == mStepIterator;
    mCalibrationFunctions.push_back(cbFunction);
    if (atStart)
      mStepIterator = mCalibrationFunctions.begin();
    else if (atEnd)
      mStepIterator = mCalibrationFunctions.end();
  }

  void CalibrationStateMachine::registerCalibrationStep(const std::string & message)
  {
    registerCalibrationStep([this, message](vtkSmartPointer<vtkPlusTransformRepository>) { finishStep(message); });
  }

  /**
  */
  void CalibrationStateMachine::finishStep(const std::string & message, bool waitProceed)
  {
    // show the Message
    mpParent->showMessageDialog(message, waitProceed);
    mpParent->showStatusMessage(message);
    // go to the next step
    ++mStepIterator;
    // check if this next step exists
    bool Finished = isFinished();
    // if the next step exists, update mWaiting
    if (!Finished)
      mWaiting = waitProceed;
    // if the Timer should be removed, do so
    if (waitProceed && mTimerID != 0 || Finished)
    {
      mpParent->removeTimer(*this, mTimerID);
      mTimerID = 0;
    }
    // if finished, call Finisher
    if (Finished)
      mFinisher();
  }

  /**
  */
  INavigationSupervisor * CalibrationStateMachine::getSupervisor() const
  {
    return mpParent;
  }

  void CalibrationStateMachine::setTransformGenerator(const TransformGeneratorFunction& tg)
  {
    mTransformGenerator = tg;
  }

  const CalibrationStateMachine::TransformGeneratorFunction& CalibrationStateMachine::getTransformGenerator() const
  {
    return mTransformGenerator;
  }

  void CalibrationStateMachine::setFinisher(const FinishedFunction & ff)
  {
    mFinisher = ff;
  }

  const CalibrationStateMachine::FinishedFunction& CalibrationStateMachine::getFinisher() const
  {
    return mFinisher;
  }

  bool CalibrationStateMachine::isFinished() const
  {
    return mStepIterator == mCalibrationFunctions.end();
  }

  /**
  */
  void CalibrationStateMachine::setSupervisor(INavigationSupervisor * parent)
  {
    mpParent = parent;
  }

  /**
  */
  PlusStatus CalibrationStateMachine::ReadConfiguration(vtkSmartPointer<vtkXMLDataElement> elem)
  {
    if (elem == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE(
        (std::string("XMLDataElement passed to ") + __FUNCTION__ + " is not vaild.").c_str())
    }

    auto pCalibrationAlgoElem = elem->FindNestedElementWithName(name());
    auto pAttribute = pCalibrationAlgoElem->GetAttribute("UpdateTimer");
    if (pAttribute != NULL)
    {
      std::istringstream ss(pAttribute);
      boost::chrono::milliseconds a;
      // read UpdateTimer from XML
      ss >> a;
      // convert UpdateTimer from boost to std (using milliseconds)
      mTimerInterval = std::chrono::milliseconds(a.count());
    }
    {
      int temp;
      if (pCalibrationAlgoElem->GetScalarAttribute("CountDownTicks", temp))
        setCountDownTicks(temp);
    }

/*    pAttribute = elem->GetAttribute("SourceTool");
    if (pAttribute != NULL)
    {
      mSourceTool = pAttribute;
    }
    pAttribute = elem->GetAttribute("TargetTool");
    if (pAttribute != NULL)
    {
      mTargetTool = pAttribute;
    }*/
    return PlusStatus(1);
  }

  /**
  */
  void CalibrationStateMachine::tick(INavigationSupervisor::TimerID timerid)
  {
    MUK_ASSERT_SUPERVISOR
    if (isFinished())
    {
      mpParent->removeTimer(*this, timerid);
      mTimerID = 0;
      throw MUK_EXCEPTION("Trying to execute a non-existent Calibration Function.",
        "The StepIterator is \'out of bounds\'.")
    }
    // call the current ticks function
    (*mStepIterator)(mTransformGenerator());
  }

  /**
  */
  void CalibrationStateMachine::init()
  {
    mWaiting = false;
    mStepIterator = mCalibrationFunctions.begin();
    initializeTimer();
  }

  /**
  */
  void CalibrationStateMachine::proceed()
  {
    mWaiting = false;
    if(!isFinished())
      countDownToStep();
  }

  /**
  */
  void CalibrationStateMachine::terminate()
  {
    mWaiting = false;
    if (mTimerID == 0)
      getSupervisor()->removeTimer(*this, mTimerID);
    mStepIterator = mCalibrationFunctions.end();
  }

  /**
  */
  void CalibrationStateMachine::setState(const enConsumerState state)
  {
    if (mState == state) return;
    getSupervisor()->setConsumerState(state);
    mState = state;
  }

  /**
  */
  const std::chrono::milliseconds& CalibrationStateMachine::getTimerInterval() const
  {
    return mTimerInterval;
  }

  void CalibrationStateMachine::countDownToStep()
  {
    // 
    for (int i = mCountdownTicks-1; i > 0; --i)
      getSupervisor()->runDelayed(*this, std::chrono::seconds(i),
        std::bind(&INavigationSupervisor::showStatusMessage, getSupervisor(), 
          (boost::format("Next step of %s is starting in %d seconds...") % name() % i).str() )
        );
    getSupervisor()->runDelayed(*this, std::chrono::seconds(mCountdownTicks), 
      std::bind(&CalibrationStateMachine::initializeTimer, this));
  }

  /**
  */
  void CalibrationStateMachine::initializeTimer()
  {
    // do this with Observers?

    if (mpParent == nullptr)
    {
      throw MUK_EXCEPTION_SIMPLE("CalibrationMachine does not have a INavigationSupervisor object set.")
    }
    if (mTimerID != 0)
      mpParent->removeTimer(*this, mTimerID);
    mTimerID = mpParent->addTimer(*this, mTimerInterval,
      std::bind(&CalibrationStateMachine::tick, this, std::placeholders::_1));
  }
}
}