#include "private/muk.pch"
#include "muk_colors.h"

#include "VisCoordinateSystemCollection.h"

#include "MukCommon/muk_common.h"
#include "MukCommon/MukTransform.h"

#include <vtkSmartPointer.h>

//#include <vtkAxesActor.h>
//#include <vtkCaptionActor2D.h>
//#include <vtkTextProperty.h>
//#include <vtkTransform.h>

namespace gris
{
namespace muk
{

  /**
  */
  VisCoordinateSystemCollection::VisCoordinateSystemCollection()
    : VisCoordinateSystemCollection(
      VisTimeDelays())
/*      {  // does not work
        std::chrono::seconds(30), // fade
        std::chrono::seconds(30), // stop fade
        std::chrono::seconds(30) }) // hide*/
  {   }

  /**
  */
  VisCoordinateSystemCollection::VisCoordinateSystemCollection(const VisTimeDelays& timedelays)
    : mTimeDelays(timedelays)
    , mLastSearchValid(false)
    , mLastSearchName()
    , mLastSearchIterator()
    , mTransforms()
  { 
    mTransforms.reserve(10); 
  /*  : mTimeToFade(0)
      , mTimeToHide(0)
      , mTimeToStopFade(0)
      declareProperty<double>("Coordinate System: Time To Fade",
        std::bind(&VisualizationLogic::setCameraConfiguration, this, std::placeholders::_1),
        std::bind(&VisualizationLogic::getCameraConfiguration, this)); */
  }

  /**
  */
  VisCoordinateSystemCollection::~VisCoordinateSystemCollection()
  {
    // all VisCoordinateSystem Object will remove themselves from their Renderer
    // upon their destruction
  }

  /** dynamic Properties...
  */
  void VisCoordinateSystemCollection::update()
  {
  }

  void VisCoordinateSystemCollection::updateBeforeRender()
  {
    typedef std::chrono::milliseconds Time;
    if (!mTimeDelays.any()) return; // no effects
    auto ttfade = mTimeDelays.getTimeToFade<Time>();
    auto ttstopfade = mTimeDelays.getTimeToStopFade<Time>();
    auto tthide = mTimeDelays.getTimeToHide<Time>();
    auto _now = clock::now();
    auto _now_minus_ttfade = _now - ttfade;
    auto _now_minus_ttstopfade = _now - ttstopfade;
    auto _now_minus_tthide = _now - tthide;

    // since we will need a lock for the mutex most times, just aquire now
    LockType lock(mTransformsMutex);

    // do a fade?
    if (ttfade != Time::zero()
      && ttfade < ttstopfade
      && ttfade < tthide)
    {
      for (auto& currentTransform : mTransforms)
      {
        if (!currentTransform.second->isVisible()) continue;
        auto acqtime = currentTransform.first.getAcquistionTime();
        if (acqtime < _now_minus_ttfade) // before fade start
          continue;
        if (acqtime < _now_minus_ttstopfade) // fade not finished
          currentTransform.second->fade((_now - acqtime) / (ttstopfade - ttfade));
        else // fade finished
          currentTransform.second->fade(1);
      }
    }
    if (tthide != Time::zero())
    {
      for (auto& currentTransform : mTransforms)
      {
        if (!currentTransform.second->isVisible()) continue;
        if (currentTransform.first.getAcquistionTime() > _now_minus_tthide)
          currentTransform.second->setVisibility(false);
      }
    }
  }

  /**
  */
  void VisCoordinateSystemCollection::setRenderer(vtkRenderer * pRenderer)
  {
    // aquire recursive mutex
    LockType lock(mTransformsMutex);
    // change the Renderer for all "children"
    std::for_each(mTransforms.begin(), mTransforms.end(),
      [pRenderer](CoordinateSystemPair& pair) { pair.second->setRenderer(pRenderer); });
    VisualObject::setRenderer(pRenderer);
  }

  /**
  */
  void VisCoordinateSystemCollection::updateCoordinateSystem(const MukTransform & transform)
  {
    // aquire recursive mutex
    LockType lock(mTransformsMutex);
    // search the Coordinate System Objects in the internal data structure
    // findCoordinateSystem also sets the internal mLastSearchIterator 'pointer' to
    // found Transforms Name >= name.
    auto itemfound = findCoordinateSystem(transform.getName());
    // if the passed Transform's name is not found, add it (this invalidates 
    // stored iteratators --> mSearchUpToDate = false)
    if (!itemfound)
    {
      // if the LastSearch is not Valid, findCoordinateSystem could not find a transform,
      // whose name is >= the requested name --> add the new transfrom at the end.
      if (!mLastSearchValid)
        mLastSearchIterator = mTransforms.end();
      // create the VisCoordinateSystem
      auto pCS = std::make_unique<VisCoordinateSystem>();    // create a new 
      // initialize VisCoordinateSystem
      pCS->setRenderer(mpRenderer);
      pCS->setName(transform);
      pCS->setTransform(transform);
      // Update search with recently added iterator, so it is also valid
      mLastSearchName = transform.getName();
      mLastSearchValid = true;
      // otherwise the object will be inserted just before the currently stored element
      // which is the transform, that is just larger than the passed transform
      mLastSearchIterator = mTransforms.insert(
        mLastSearchIterator,
        std::make_pair<CoordinateSystemPair::first_type, CoordinateSystemPair::second_type>(
          MukTransform(transform), // copy the transform into the pair
          nullptr                  // nullptr, so we can swap the other value into here...
          )
      );
      mLastSearchIterator->second.swap(pCS);
    }
    else
    {
      // mLastSearch is not invalidated, just updated
      mLastSearchIterator->first = transform;
      mLastSearchIterator->second->setTransform(transform);
    }
    mLastSearchIterator->second->setVisibility(true);
  }

  /**
  */
  void VisCoordinateSystemCollection::deleteCoordinateSystem(const std::string & name)
  {
    // aquire recursive mutex
    LockType lock(mTransformsMutex);
    // find the CS ...
    auto itemfound = findCoordinateSystem(name);
    if (itemfound)
    {
      // delete the CS, invalidating iterators
      mTransforms.erase(mLastSearchIterator);
      mLastSearchValid = false;
    }
  }

  /**
  */
  void VisCoordinateSystemCollection::clearCoordinateSystems()
  {
    // aquire recursive mutex
    LockType lock(mTransformsMutex);
    // deleting all Transforms
    mLastSearchValid = false;
    mTransforms.clear();
  }

  /** tries to find the coordinate system, whose name is == the passed name
  *   returns true, if the passed transform is found, also internally sets the mLastSearchIterator 
  *       to this element
  *   if the passed name is not found, tries to set the interal mLastSearchIterator to the next 
  *       element (whose name shold be > than the passed name)
  *   if no such element is found, sets the iterator to end()
  */
  bool VisCoordinateSystemCollection::findCoordinateSystem(const std::string & name)
  {
    // aquire recursive mutex
    LockType lock(mTransformsMutex);
    // do we need to update our Search?
    if (!mLastSearchValid || name != mLastSearchName)
    {
      mLastSearchValid = false;
      // lower_bound returns the >= value, i.e. the correct Transform or the next-in-line
      // no transform is >= :    -> end()
      // all transforms are >= : -> begin()
      mLastSearchIterator = std::lower_bound(mTransforms.begin(), mTransforms.end(), name,
        [](const CoordinateSystemPair& p, const std::string& n) { return p.first.getName() < n; });
      // mLastSearchIterator will be mTransform.end(), if the new transform is < all other transforms
      mLastSearchValid = mLastSearchIterator != mTransforms.end();
      if (mLastSearchValid) // only update the Name, if the iterator is actually valid
        mLastSearchName  = mLastSearchIterator->first.getName();
    }
    return mLastSearchName == name;
  }

}
}